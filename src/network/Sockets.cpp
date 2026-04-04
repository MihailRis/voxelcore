#include "commons.hpp"

#pragma comment(lib, "Ws2_32.lib")

#define NOMINMAX
#include <stdexcept>
#include <limits>
#include <queue>
#include <thread>

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
                logger.warning() << "an error ocurred while receiving from "
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

    int recv(char* buffer, size_t length) override {
        std::lock_guard lock(mutex);

        if (state != ConnectionState::CONNECTED && readBatch.empty()) {
            return -1;
        }
        int size = std::min(readBatch.size(), length);
        std::memcpy(buffer, readBatch.data(), size);
        readBatch.erase(readBatch.begin(), readBatch.begin() + size);
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
