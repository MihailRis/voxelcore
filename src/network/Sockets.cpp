#include "commons.hpp"

#pragma comment(lib, "Ws2_32.lib")

#define NOMINMAX
#include <stdexcept>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_map>

#ifdef _WIN32
#include <curl/curl.h>
#define SHUT_RDWR SD_BOTH
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

using SOCKET = int;
#endif // _WIN32

#include "Network.hpp"
#include "util/stringutil.hpp"
#include "debug/Logger.hpp"

using namespace network;

static debug::Logger logger("sockets");

#ifndef _WIN32
static inline int closesocket(int descriptor) noexcept {
    return close(descriptor);
}
static inline std::runtime_error handle_socket_error(const std::string& message) {
    int err = errno;
    return std::runtime_error(
        message+" [errno=" + std::to_string(err) + "]: " + 
        std::string(strerror(err))
    );
}
#else
static inline std::runtime_error handle_socket_error(const std::string& message) {
    int errorCode = WSAGetLastError();
    wchar_t* s = nullptr;
    size_t size = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        errorCode,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPWSTR)&s,
        0,
        nullptr
    );
    assert(s != nullptr);
    while (size && isspace(s[size-1])) {
        s[--size] = 0;
    }
    auto errorString = util::wstr2str_utf8(std::wstring(s));
    LocalFree(s);
    return std::runtime_error(message+" [WSA error=" + 
           std::to_string(errorCode) + "]: "+errorString);
}
#endif

static inline int connectsocket(
    int descriptor, const sockaddr* addr, socklen_t len
) noexcept {
    return connect(descriptor, addr, len);
}

static inline int recvsocket(
    int descriptor, char* buf, size_t len
) noexcept {
    return recv(descriptor, buf, len, 0);
}

static inline int sendsocket(
    int descriptor, const char* buf, size_t len, int flags
) noexcept {
    return send(descriptor, buf, len, flags);
}

static std::string to_string(const sockaddr_in& addr, bool port=true) {
    char ip[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN)) {
        return std::string(ip) +
               (port ? (":" + std::to_string(htons(addr.sin_port))) : "");
    }
    return "";
}

class SocketTcpConnection : public TcpConnection {
    SOCKET descriptor;
    sockaddr_in addr;
    size_t totalUpload = 0;
    size_t totalDownload = 0;
    ConnectionState state = ConnectionState::INITIAL;
    std::unique_ptr<std::thread> thread = nullptr;
    std::vector<char> readBatch;
    util::Buffer<char> buffer;
    std::mutex mutex;
    std::string errorMessage;

    void connectSocket() {
        state = ConnectionState::CONNECTING;
        logger.info() << "connecting to " << to_string(addr);
        int res = connectsocket(descriptor, (const sockaddr*)&addr, sizeof(sockaddr_in));
        if (res < 0) {
            auto error = handle_socket_error("Connect failed");
            closesocket(descriptor);
            state = ConnectionState::CLOSED;
            errorMessage = error.what();
            logger.error() << errorMessage;
            return;
        }
        logger.info() << "connected to " << to_string(addr);
        state = ConnectionState::CONNECTED;
    }
public:
    SocketTcpConnection(SOCKET descriptor, sockaddr_in addr)
        : descriptor(descriptor), addr(std::move(addr)), buffer(16'384) {}

    ~SocketTcpConnection() {
        if (state != ConnectionState::CLOSED) {
            shutdown(descriptor, 2);
        }
        if (thread) {
            thread->join();
        }
    }

    void setNoDelay(bool noDelay) override {
        int opt = noDelay ? 1 : 0;
        if (setsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(opt)) < 0) {
            throw handle_socket_error("setsockopt(TCP_NODELAY) failed");
        }
    }

    bool isNoDelay() const override {
        int opt = 0;
        socklen_t len = sizeof(opt);
        if (getsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, &len) < 0) {
            throw handle_socket_error("getsockopt(TCP_NODELAY) failed");
        }
        return opt != 0;
    }

    void startListen() {
        while (state == ConnectionState::CONNECTED) {
            int size = recvsocket(descriptor, buffer.data(), buffer.size());
            if (size == 0) {
                logger.info() << "closed connection with " << to_string(addr);
                closesocket(descriptor);
                state = ConnectionState::CLOSED;
                break;
            } else if (size < 0) {
                logger.warning() << "an error occurred while receiving from "
                            << to_string(addr);
                auto error = handle_socket_error("recv(...) error");
                closesocket(descriptor);
                state = ConnectionState::CLOSED;
                logger.error() << error.what();
                break;
            }
            {
                std::lock_guard lock(mutex);
                for (size_t i = 0; i < size; i++) {
                    readBatch.emplace_back(buffer[i]);
                }
                totalDownload += size;
            }
        }
    }

    void startClient() {
        state = ConnectionState::CONNECTED;
        thread = std::make_unique<std::thread>([this]() { startListen();});
    }

    void connect(runnable callback, stringconsumer errorCallback) override {
        thread = std::make_unique<std::thread>([this, callback, errorCallback]() {
            connectSocket();
            if (state == ConnectionState::CONNECTED) {
                callback();
                startListen();
            } else {
                errorCallback(errorMessage);
            }
        });
    }

    int read(char* buffer, size_t length) {
        if (state != ConnectionState::CONNECTED && readBatch.empty()) {
            return -1;
        }
        int size = std::min(readBatch.size(), length);
        std::memcpy(buffer, readBatch.data(), size);
        return size;
    }

    int peek(char* buffer, size_t length) override {
        std::lock_guard lock(mutex);
        return read(buffer, length);
    }

    int recv(char* buffer, size_t length) override {
        std::lock_guard lock(mutex);
        int size = read(buffer, length);
        if (size != -1) {
            readBatch.erase(readBatch.begin(), readBatch.begin() + size);
        }
        return size;
    }

    int send(const char* buffer, size_t length) override {
        if (state == ConnectionState::CLOSED) {
            return 0;
        }
        int len = sendsocket(descriptor, buffer, length, 0);
        if (len == -1) {
            int err = errno;
            close();
            throw std::runtime_error(
                "Send failed [errno=" + std::to_string(err) + "]: "
                 + std::string(strerror(err))
            );
        }
        totalUpload += len;
        return len;
    }

    int available() override {
        std::lock_guard lock(mutex);
        return readBatch.size();
    }

    void close(bool discardAll=false) override {
        {
            std::lock_guard lock(mutex);
            readBatch.clear();

            if (state != ConnectionState::CLOSED) {
                shutdown(descriptor, SHUT_RDWR);
                closesocket(descriptor);
            }
        }
        if (thread) {
            thread->join();
            thread = nullptr;
        }
    }

    size_t pullUpload() override {
        size_t size = totalUpload;
        totalUpload = 0;
        return size;
    }

    size_t pullDownload() override {
        size_t size = totalDownload;
        totalDownload = 0;
        return size;
    }

    int getPort() const override {
        return htons(addr.sin_port);
    }

    std::string getAddress() const override {
        return to_string(addr, false);
    }

    static std::shared_ptr<SocketTcpConnection> connect(
        const std::string& address,
        int port,
        runnable callback,
        stringconsumer errorCallback
    ) {
        addrinfo hints {};

        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo* addrinfo = nullptr;
        if (int res = getaddrinfo(
            address.c_str(), nullptr, &hints, &addrinfo
        )) {
            std::string errorMessage = gai_strerror(res);
            if (errorCallback) {
                errorCallback(errorMessage);
            }
            throw std::runtime_error(errorMessage);
        }

        sockaddr_in serverAddress;
        std::memcpy(&serverAddress, addrinfo->ai_addr, sizeof(sockaddr_in));
        serverAddress.sin_port = htons(port);
        freeaddrinfo(addrinfo);

        SOCKET descriptor = socket(AF_INET, SOCK_STREAM, 0);
        if (descriptor == -1) {
            std::string errorMessage = "could not create socket";
            if (errorCallback) {
                errorCallback(errorMessage);
            }
            throw std::runtime_error(errorMessage);
        }
        auto socket = std::make_shared<SocketTcpConnection>(descriptor, std::move(serverAddress));
        socket->connect(std::move(callback), std::move(errorCallback));
        return socket;
    }

    ConnectionState getState() const override {
        return state;
    }
};

class SocketTcpServer : public TcpServer {
    u64id_t id;
    Network* network;
    SOCKET descriptor;
    std::vector<u64id_t> clients;
    std::mutex clientsMutex;
    bool open = true;
    std::unique_ptr<std::thread> thread = nullptr;
    int port;
    int maxConnected = -1;
public:
    SocketTcpServer(u64id_t id, Network* network, SOCKET descriptor, int port)
    : id(id), network(network), descriptor(descriptor), port(port) {}

    ~SocketTcpServer() {
        closeSocket();
    }

    void setMaxClientsConnected(int count) override {
        maxConnected = count;
    }

    void update() override {
        std::vector<u64id_t> clients;
        for (u64id_t cid : this->clients) {
            if (auto client = network->getConnection(cid, true)) {
                if (client->getState() != ConnectionState::CLOSED) {
                    clients.emplace_back(cid);
                }
            }
        }
        std::swap(clients, this->clients);
    }

    void startListen(ConnectCallback handler) override {
        thread = std::make_unique<std::thread>([this, handler]() {
            while (open) {
                logger.info() << "listening for connections";
                if (listen(descriptor, 2) < 0) {
                    close();
                    break;
                }
                socklen_t addrlen = sizeof(sockaddr_in);
                SOCKET clientDescriptor;
                sockaddr_in address;
                logger.info() << "accepting clients";
                if ((clientDescriptor = accept(descriptor, (sockaddr*)&address, &addrlen)) == -1) {
                    close();
                    break;
                }
                if (maxConnected >= 0 && clients.size() >= maxConnected) {
                    logger.info() << "refused connection attempt from " << to_string(address);
                    closesocket(clientDescriptor);
                    continue;
                }
                logger.info() << "client connected: " << to_string(address);
                auto socket = std::make_shared<SocketTcpConnection>(
                    clientDescriptor, address
                );
                socket->startClient();
                u64id_t id = network->addConnection(socket);
                {
                    std::lock_guard lock(clientsMutex);
                    clients.push_back(id);
                }
                handler(this->id, id);
            }
        });
    }
    
    void closeSocket() {
        if (!open) {
            return;
        }
        logger.info() << "closing server";
        open = false;

        {
            std::lock_guard lock(clientsMutex);
            for (u64id_t clientid : clients) {
                if (auto client = network->getConnection(clientid, true)) {
                    client->close();
                }
            }
        }
        clients.clear();

        shutdown(descriptor, 2);
        closesocket(descriptor);
        thread->join();
    }

    void close() override {
        closeSocket();
    }
    
    bool isOpen() override {
        return open;
    }

    int getPort() const override {
        return port;
    }

    static std::shared_ptr<SocketTcpServer> openServer(
        u64id_t id, Network* network, int port, ConnectCallback handler
    ) {
        SOCKET descriptor = socket(
            AF_INET, SOCK_STREAM, 0
        );
        if (descriptor == -1) {
            throw std::runtime_error("Could not create server socket");
        }
        int opt = 1;
        int flags = SO_REUSEADDR;
#       if !defined(_WIN32) && !defined(__APPLE__)
            flags |= SO_REUSEPORT;
#       endif
        if (setsockopt(descriptor, SOL_SOCKET, flags, (const char*)&opt, sizeof(opt))) {
            logger.error() << "setsockopt(SO_REUSEADDR) failed with errno: "
             << errno << "(" << std::strerror(errno) << ")";
            closesocket(descriptor);
            throw std::runtime_error("setsockopt");
        }
        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        if (bind(descriptor, (sockaddr*)&address, sizeof(address)) < 0) {
            closesocket(descriptor);
            throw std::runtime_error("could not bind port "+std::to_string(port));
        }
        port = ntohs(address.sin_port);
        logger.info() << "opened server at port " << port;
        auto server =
            std::make_shared<SocketTcpServer>(id, network, descriptor, port);
        server->startListen(std::move(handler));
        return server;
    }
};

static sockaddr_in resolve_address_dgram(const std::string& address, int port) {
    sockaddr_in serverAddr{};
    addrinfo hints {};

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo* addrinfo = nullptr;
    if (int res = getaddrinfo(
        address.c_str(), nullptr, &hints, &addrinfo
    )) {
        throw std::runtime_error(gai_strerror(res));
    }

    std::memcpy(&serverAddr, addrinfo->ai_addr, sizeof(sockaddr_in));
    serverAddr.sin_port = htons(port);
    freeaddrinfo(addrinfo);
    return serverAddr;
}

class SocketUdpConnection : public UdpConnection {
    u64id_t id;
    SOCKET descriptor;
    sockaddr_in addr{};
    bool open = true;
    std::unique_ptr<std::thread> thread;
    ClientDatagramCallback callback;

    size_t totalUpload = 0;
    size_t totalDownload = 0;
    ConnectionState state = ConnectionState::INITIAL;

public:
    SocketUdpConnection(u64id_t id, SOCKET descriptor, sockaddr_in addr)
        : id(id), descriptor(descriptor), addr(std::move(addr)) {}

    ~SocketUdpConnection() override {
        SocketUdpConnection::close();
    }

    static std::shared_ptr<SocketUdpConnection> connect(
        u64id_t id,
        const std::string& address,
        int port,
        ClientDatagramCallback handler,
        runnable callback
    ) {
        SOCKET descriptor = socket(AF_INET, SOCK_DGRAM, 0);
        if (descriptor == -1) {
            throw std::runtime_error("could not create udp socket");
        }

        sockaddr_in serverAddr = resolve_address_dgram(address, port);

        if (::connect(descriptor, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            auto err = handle_socket_error("udp connect failed");
            closesocket(descriptor);
            throw err;
        }

        auto socket = std::make_shared<SocketUdpConnection>(id, descriptor, serverAddr);
        socket->connect(std::move(handler));

        callback();

        return socket;
    }

    void connect(ClientDatagramCallback handler) override {
        callback = std::move(handler);
        state = ConnectionState::CONNECTED;

        thread = std::make_unique<std::thread>([this]() {
            util::Buffer<char> buffer(16'384);
            while (open) {
                int size = recv(descriptor, buffer.data(), buffer.size(), 0);
                if (size <= 0) {
                    logger.error() << "udp connection " << id
                                   << handle_socket_error(" recv error").what();
                    if (!open) break;
                    closesocket(descriptor);
                    state = ConnectionState::CLOSED;
                    break;
                }
                totalDownload += size;
                if (callback) {
                    callback(id, buffer.data(), size);
                }
            }
        });
    }

    int send(const char* buffer, size_t length) override {
        int len = ::send(descriptor, buffer, length, 0);
        if (len < 0) {
            auto err = handle_socket_error(" send failed");
            closesocket(descriptor);
            state = ConnectionState::CLOSED;
            logger.error() << "udp connection " << id << err.what();
        } else totalUpload += len;

        return len;
    }

    void close(bool discardAll=false) override {
        if (!open) return;
        open = false;
        logger.info() << "closing udp connection "<< id;

        if (state != ConnectionState::CLOSED) {
            shutdown(descriptor, 2);
            closesocket(descriptor);
        }

        if (thread) {
            thread->join();
            thread.reset();
        }
        state = ConnectionState::CLOSED;
    }

    size_t pullUpload() override {
        size_t s = totalUpload;
        totalUpload = 0;
        return s;
    }

    size_t pullDownload() override {
        size_t s = totalDownload;
        totalDownload = 0;
        return s;
    }

    [[nodiscard]] int getPort() const override {
        return ntohs(addr.sin_port);
    }

    [[nodiscard]] std::string getAddress() const override {
        return to_string(addr, false);
    }

    [[nodiscard]] ConnectionState getState() const override {
        return state;
    }
};

class SocketUdpServer : public UdpServer {
    u64id_t id;
    SOCKET descriptor;
    bool open = true;
    std::unique_ptr<std::thread> thread = nullptr;
    int port;
    ServerDatagramCallback callback;

public:
    SocketUdpServer(u64id_t id, Network* network, SOCKET descriptor, int port)
        : id(id), descriptor(descriptor), port(port) {}

    ~SocketUdpServer() override {
        SocketUdpServer::close();
    }

    void update() override {}

    void startListen(ServerDatagramCallback handler) override {
        callback = std::move(handler);

        thread = std::make_unique<std::thread>([this]() {
            util::Buffer<char> buffer(16384);
            sockaddr_in clientAddr{};
            socklen_t addrlen = sizeof(clientAddr);

            while (open) {
                int size = recvfrom(descriptor, buffer.data(), buffer.size(), 0,
                                    reinterpret_cast<sockaddr*>(&clientAddr), &addrlen);
                if (size <= 0) {
                    if (!open) break;
                    continue;
                }

                std::string addrStr = to_string(clientAddr, false);
                int port = ntohs(clientAddr.sin_port);

                callback(id, addrStr, port, buffer.data(), size);
            }
        });
    }

    void sendTo(const std::string& addr, int port, const char* buffer, size_t length) override {
        sockaddr_in client = resolve_address_dgram(addr, port);
        if (sendto(descriptor, buffer, length, 0,
               reinterpret_cast<sockaddr*>(&client), sizeof(client)) < 0) {
            logger.error() << handle_socket_error("sendto").what();
        }
    }

    void close() override {
        if (!open) return;
        open = false;
        shutdown(descriptor, 2);
        closesocket(descriptor);
        if (thread) {
            thread->join();
            thread = nullptr;
        }
    }

    bool isOpen() override { return open; }
    int getPort() const override { return port; }

    static std::shared_ptr<SocketUdpServer> openServer(
        u64id_t id, Network* network, int port, const ServerDatagramCallback& handler
    ) {
        SOCKET descriptor = socket(AF_INET, SOCK_DGRAM, 0);
        if (descriptor == -1) throw std::runtime_error("could not create udp socket");

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(descriptor, (sockaddr*)&address, sizeof(address)) < 0) {
            closesocket(descriptor);
            throw std::runtime_error("could not bind udp port " + std::to_string(port));
        }

        auto server = std::make_shared<SocketUdpServer>(id, network, descriptor, port);
        server->startListen(std::move(handler));
        return server;
    }
};

class SocketHttpServer;

namespace {
    constexpr size_t HTTP_MAX_HEADER_SIZE = 32 * 1024;
    constexpr size_t HTTP_MAX_BODY_SIZE = 16 * 1024 * 1024;

    std::string url_decode(std::string_view s) {
        std::string result;
        result.reserve(s.size());
        for (size_t i = 0; i < s.size(); i++) {
            if (s[i] == '%' && i + 2 < s.size()) {
                auto hex = std::string(s.substr(i + 1, 2));
                char* end = nullptr;
                long code = std::strtol(hex.c_str(), &end, 16);
                if (end == hex.c_str() + 2) {
                    result.push_back(static_cast<char>(code));
                    i += 2;
                    continue;
                }
            }
            result.push_back(s[i] == '+' ? ' ' : s[i]);
        }
        return result;
    }

    const char* http_reason_phrase(int status) {
        switch (status) {
            case 200: return "OK";
            case 201: return "Created";
            case 202: return "Accepted";
            case 204: return "No Content";
            case 301: return "Moved Permanently";
            case 302: return "Found";
            case 304: return "Not Modified";
            case 400: return "Bad Request";
            case 401: return "Unauthorized";
            case 403: return "Forbidden";
            case 404: return "Not Found";
            case 405: return "Method Not Allowed";
            case 408: return "Request Timeout";
            case 411: return "Length Required";
            case 413: return "Payload Too Large";
            case 431: return "Request Header Fields Too Large";
            case 500: return "Internal Server Error";
            case 501: return "Not Implemented";
            case 503: return "Service Unavailable";
            default: return "Unknown";
        }
    }

    bool http_header_has(const std::vector<std::string>& headers, const std::string& name) {
        auto lname = util::lower_case(name);
        for (const auto& header : headers) {
            if (header.find(':') == std::string::npos) continue;
            auto [hname, hvalue] = util::split_at(header, ':');
            util::trim(hname);
            if (util::lower_case(hname) == lname) {
                return true;
            }
        }
        return false;
    }

    std::string build_http_response(const HttpServerResponse& response) {
        std::string out;
        out += "HTTP/1.1 " + std::to_string(response.status) + " " +
               http_reason_phrase(response.status) + "\r\n";
        for (const auto& header : response.headers) {
            out += header + "\r\n";
        }
        if (!http_header_has(response.headers, "Content-Length")) {
            out += "Content-Length: " + std::to_string(response.body.size()) + "\r\n";
        }
        if (!http_header_has(response.headers, "Connection")) {
            out += "Connection: close\r\n";
        }
        out += "\r\n";
        out += response.body;
        return out;
    }

    struct PendingHttpRequest {
        std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        HttpServerResponse response;
    };

    bool http_recv_more(SOCKET descriptor, std::string& buffer) {
        char chunk[4096];
        int size = recvsocket(descriptor, chunk, sizeof(chunk));
        if (size <= 0) {
            return false;
        }
        buffer.append(chunk, size);
        return true;
    }

    void http_send_all(SOCKET descriptor, const std::string& data) {
        size_t sent = 0;
        while (sent < data.size()) {
            int len = sendsocket(
                descriptor, data.data() + sent, data.size() - sent, 0
            );
            if (len <= 0) {
                return;
            }
            sent += static_cast<size_t>(len);
        }
    }

    void handle_http_client(
        SOCKET descriptor,
        sockaddr_in addr,
        u64id_t serverId,
        std::shared_ptr<SocketHttpServer> server,
        HttpRequestCallback handler
    );
}

class SocketHttpServer
    : public HttpServer, public std::enable_shared_from_this<SocketHttpServer> {
    u64id_t id;
    SOCKET descriptor;
    int port;
    std::atomic<bool> open {true};
    std::unique_ptr<std::thread> thread = nullptr;

    std::mutex pendingMutex;
    std::unordered_map<u64id_t, std::shared_ptr<PendingHttpRequest>> pending;
    u64id_t nextRequestId = 1;
    long responseTimeoutMs;
public:
    SocketHttpServer(u64id_t id, SOCKET descriptor, int port, long responseTimeoutMs)
    : id(id), descriptor(descriptor), port(port), responseTimeoutMs(responseTimeoutMs) {}

    [[nodiscard]] long getResponseTimeoutMs() const {
        return responseTimeoutMs;
    }

    ~SocketHttpServer() {
        closeSocket();
    }

    void update() override {}

    void startListen(HttpRequestCallback handler) override {
        thread = std::make_unique<std::thread>([this, handler]() {
            while (open) {
                logger.info() << "listening for http connections";
                if (listen(descriptor, 16) < 0) {
                    close();
                    break;
                }
                socklen_t addrlen = sizeof(sockaddr_in);
                SOCKET clientDescriptor;
                sockaddr_in address;
                if ((clientDescriptor = accept(descriptor, (sockaddr*)&address, &addrlen)) == -1) {
                    close();
                    break;
                }
                logger.info() << "http client connected: " << to_string(address);
                std::thread(
                    handle_http_client, clientDescriptor, address, id,
                    shared_from_this(), handler
                ).detach();
            }
        });
    }

    u64id_t registerPending(const std::shared_ptr<PendingHttpRequest>& request) {
        std::lock_guard lock(pendingMutex);
        u64id_t requestId = nextRequestId++;
        pending[requestId] = request;
        return requestId;
    }

    void unregisterPending(u64id_t requestId) {
        std::lock_guard lock(pendingMutex);
        pending.erase(requestId);
    }

    void respond(u64id_t requestId, HttpServerResponse response) override {
        std::shared_ptr<PendingHttpRequest> request;
        {
            std::lock_guard lock(pendingMutex);
            auto found = pending.find(requestId);
            if (found == pending.end()) {
                return;
            }
            request = found->second;
        }
        {
            std::lock_guard lock(request->mutex);
            request->response = std::move(response);
            request->done = true;
        }
        request->cv.notify_all();
    }

    void closeSocket() {
        if (!open) {
            return;
        }
        logger.info() << "closing http server";
        open = false;

        shutdown(descriptor, 2);
        closesocket(descriptor);
        if (thread) {
            thread->join();
            thread = nullptr;
        }
    }

    void close() override {
        closeSocket();
    }

    bool isOpen() override {
        return open;
    }

    int getPort() const override {
        return port;
    }

    static std::shared_ptr<SocketHttpServer> openServer(
        u64id_t id,
        Network* network,
        int port,
        HttpRequestCallback handler,
        long responseTimeoutMs
    ) {
        SOCKET descriptor = socket(
            AF_INET, SOCK_STREAM, 0
        );
        if (descriptor == -1) {
            throw std::runtime_error("Could not create http server socket");
        }
        int opt = 1;
        int flags = SO_REUSEADDR;
#       if !defined(_WIN32) && !defined(__APPLE__)
            flags |= SO_REUSEPORT;
#       endif
        if (setsockopt(descriptor, SOL_SOCKET, flags, (const char*)&opt, sizeof(opt))) {
            logger.error() << "setsockopt(SO_REUSEADDR) failed with errno: "
             << errno << "(" << std::strerror(errno) << ")";
            closesocket(descriptor);
            throw std::runtime_error("setsockopt");
        }
        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        if (bind(descriptor, (sockaddr*)&address, sizeof(address)) < 0) {
            closesocket(descriptor);
            throw std::runtime_error("could not bind port "+std::to_string(port));
        }
        socklen_t len = sizeof(address);
        getsockname(descriptor, (sockaddr*)&address, &len);
        port = ntohs(address.sin_port);
        logger.info() << "opened http server at port " << port;
        auto server = std::make_shared<SocketHttpServer>(
            id, descriptor, port, responseTimeoutMs
        );
        server->startListen(std::move(handler));
        return server;
    }
};

namespace {
    void handle_http_client(
        SOCKET descriptor,
        sockaddr_in addr,
        u64id_t serverId,
        std::shared_ptr<SocketHttpServer> server,
        HttpRequestCallback handler
    ) {
        auto finish = [&](HttpServerResponse response) {
            http_send_all(descriptor, build_http_response(response));
            shutdown(descriptor, SHUT_RDWR);
            closesocket(descriptor);
        };

        std::string buffer;
        size_t headEnd;
        while ((headEnd = buffer.find("\r\n\r\n")) == std::string::npos) {
            if (buffer.size() > HTTP_MAX_HEADER_SIZE) {
                finish({431, {"Content-Type: text/plain"}, http_reason_phrase(431)});
                return;
            }
            if (!http_recv_more(descriptor, buffer)) {
                shutdown(descriptor, SHUT_RDWR);
                closesocket(descriptor);
                return;
            }
        }

        std::string head = buffer.substr(0, headEnd);
        std::string rest = buffer.substr(headEnd + 4);

        size_t lineEnd = head.find("\r\n");
        std::string requestLine =
            head.substr(0, lineEnd == std::string::npos ? head.size() : lineEnd);

        auto tokens = util::split(requestLine, ' ');
        std::string method = tokens.size() > 0 ? tokens[0] : "";
        std::string target = tokens.size() > 1 ? tokens[1] : "";

        if (method.empty() || target.empty()) {
            finish({400, {"Content-Type: text/plain"}, http_reason_phrase(400)});
            return;
        }

        std::string path = target;
        std::string query;
        if (auto qpos = target.find('?'); qpos != std::string::npos) {
            path = target.substr(0, qpos);
            query = target.substr(qpos + 1);
        }
        path = url_decode(path);

        std::vector<std::string> headers;
        std::string contentLength;
        std::string transferEncoding;

        size_t pos = lineEnd == std::string::npos ? head.size() : lineEnd + 2;
        while (pos < head.size()) {
            size_t next = head.find("\r\n", pos);
            if (next == std::string::npos) next = head.size();
            std::string line = head.substr(pos, next - pos);
            pos = next + 2;
            if (line.empty() || line.find(':') == std::string::npos) continue;

            auto [name, value] = util::split_at(line, ':');
            util::trim(name);
            util::trim(value);
            headers.push_back(name + ": " + value);

            auto lname = util::lower_case(name);
            if (lname == "content-length") {
                contentLength = value;
            } else if (lname == "transfer-encoding") {
                transferEncoding = util::lower_case(value);
            }
        }

        if (transferEncoding.find("chunked") != std::string::npos) {
            finish({501, {"Content-Type: text/plain"}, http_reason_phrase(501)});
            return;
        }

        size_t bodyLength = 0;
        if (!contentLength.empty()) {
            try {
                bodyLength = std::stoull(contentLength);
            } catch (...) {
                finish({400, {"Content-Type: text/plain"}, http_reason_phrase(400)});
                return;
            }
        }

        if (bodyLength > HTTP_MAX_BODY_SIZE) {
            finish({413, {"Content-Type: text/plain"}, http_reason_phrase(413)});
            return;
        }

        while (rest.size() < bodyLength) {
            if (!http_recv_more(descriptor, rest)) {
                shutdown(descriptor, SHUT_RDWR);
                closesocket(descriptor);
                return;
            }
        }
        std::string body = rest.substr(0, bodyLength);

        auto pending = std::make_shared<PendingHttpRequest>();
        u64id_t requestId = server->registerPending(pending);

        HttpServerRequest request;
        request.requestId = requestId;
        request.method = std::move(method);
        request.path = std::move(path);
        request.query = std::move(query);
        request.headers = std::move(headers);
        request.body = std::move(body);
        request.remoteAddr = to_string(addr, false);
        request.remotePort = ntohs(addr.sin_port);

        handler(serverId, std::move(request));

        HttpServerResponse response;
        {
            std::unique_lock lock(pending->mutex);
            long timeoutMs = server->getResponseTimeoutMs();
            bool completed;
            if (timeoutMs > 0) {
                completed = pending->cv.wait_for(
                    lock,
                    std::chrono::milliseconds(timeoutMs),
                    [&]() { return pending->done; }
                );
            } else {
                pending->cv.wait(lock, [&]() { return pending->done; });
                completed = true;
            }
            if (completed) {
                response = std::move(pending->response);
            } else {
                response.status = 503;
                response.headers = {"Content-Type: text/plain"};
                response.body = http_reason_phrase(503);
            }
        }
        server->unregisterPending(requestId);

        finish(std::move(response));
    }
}

namespace network {
    std::shared_ptr<TcpConnection> connect_tcp(
        const std::string& address,
        int port,
        runnable callback,
        stringconsumer errorCallback
    ) {
        return SocketTcpConnection::connect(
            address, port, std::move(callback), std::move(errorCallback)
        );
    }

    std::shared_ptr<TcpServer> open_tcp_server(
        u64id_t id, Network* network, int port, ConnectCallback handler
    ) {
        return SocketTcpServer::openServer(id, network, port, std::move(handler));
    }

    std::shared_ptr<UdpConnection> connect_udp(
        u64id_t id,
        const std::string& address,
        int port,
        ClientDatagramCallback handler,
        runnable callback
    ) {
        return SocketUdpConnection::connect(
            id, address, port, std::move(handler), std::move(callback)
        );
    }

    std::shared_ptr<UdpServer> open_udp_server(
        u64id_t id,
        Network* network,
        int port,
        const ServerDatagramCallback& handler
    ) {
        return SocketUdpServer::openServer(id, network, port, handler);
    }

    std::shared_ptr<HttpServer> open_http_server(
        u64id_t id,
        Network* network,
        int port,
        HttpRequestCallback handler,
        long responseTimeoutMs
    ) {
        return SocketHttpServer::openServer(
            id, network, port, std::move(handler), responseTimeoutMs
        );
    }

    int find_free_port() {
        SOCKET descriptor = socket(AF_INET, SOCK_STREAM, 0);
        if (descriptor == -1) {
            return -1;
        }
        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = 0;
        if (bind(descriptor, (sockaddr*)&address, sizeof(address)) < 0) {
            closesocket(descriptor);
            return -1;
        }
        socklen_t len = sizeof(address);
        getsockname(descriptor, (sockaddr*)&address, &len);
        int port = ntohs(address.sin_port);
        closesocket(descriptor);
        return port;
    }
}
