#include "Network.hpp"

#pragma comment(lib, "Ws2_32.lib")

#define NOMINMAX
#include <curl/curl.h>
#include <stdexcept>
#include <limits>
#include <queue>
#include <thread>

#ifdef _WIN32
/// included in curl.h
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

#include "debug/Logger.hpp"
#include "util/stringutil.hpp"

using namespace network;

inline constexpr int HTTP_OK = 200;
inline constexpr int HTTP_BAD_GATEWAY = 502;

static debug::Logger logger("network");

static size_t write_callback(
    char* ptr, size_t size, size_t nmemb, void* userdata
) {
    auto& buffer = *reinterpret_cast<std::vector<char>*>(userdata);
    size_t psize = buffer.size();
    buffer.resize(psize + size * nmemb);
    std::memcpy(buffer.data() + psize, ptr, size * nmemb);
    return size * nmemb;
}

enum class RequestType {
    GET, POST
};

struct Request {
    RequestType type;
    std::string url;
    OnResponse onResponse;
    OnReject onReject;
    long maxSize;
    bool followLocation = false;
    std::string data;
};

class CurlRequests : public Requests {
    CURLM* multiHandle;
    CURL* curl;

    size_t totalUpload = 0;
    size_t totalDownload = 0;

    OnResponse onResponse;
    OnReject onReject;
    std::vector<char> buffer;
    std::string url;

    std::queue<Request> requests;
public:
    CurlRequests(CURLM* multiHandle, CURL* curl)
        : multiHandle(multiHandle), curl(curl) {
    }

    virtual ~CurlRequests() {
        curl_multi_remove_handle(multiHandle, curl);
        curl_easy_cleanup(curl);
        curl_multi_cleanup(multiHandle);
    }
    void get(
        const std::string& url,
        OnResponse onResponse,
        OnReject onReject,
        long maxSize
    ) override {
        Request request {
            RequestType::GET, url, onResponse, onReject, maxSize, false, ""};
        processRequest(std::move(request));
    }

    void post(
        const std::string& url,
        const std::string& data,
        OnResponse onResponse,
        OnReject onReject=nullptr,
        long maxSize=0
    ) override {
        Request request {
            RequestType::POST, url, onResponse, onReject, maxSize, false, ""};
        request.data = data;
        processRequest(std::move(request));
    }

    void processRequest(Request request) {
        if (!url.empty()) {
            requests.push(request);
            return;
        }
        onResponse = request.onResponse;
        onReject = request.onReject;
        url = request.url;

        buffer.clear();

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, request.type == RequestType::POST);
        
        curl_slist* hs = NULL;

        switch (request.type) {
            case RequestType::GET:
                break;
            case RequestType::POST: 
                hs = curl_slist_append(hs, "Content-Type: application/json");
                curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request.data.length());
                curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, request.data.c_str());
                break;
            default:
                throw std::runtime_error("not implemented");
        }
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, request.followLocation);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.81.0");
        if (request.maxSize == 0) {
            curl_easy_setopt(
                curl, CURLOPT_MAXFILESIZE, std::numeric_limits<long>::max()
            );
        } else {
            curl_easy_setopt(curl, CURLOPT_MAXFILESIZE, request.maxSize);
        }
        curl_multi_add_handle(multiHandle, curl);
        int running;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message << " (" << url << ")";
            if (onReject) {
                onReject(HTTP_BAD_GATEWAY);
            }
            url = "";
        }
    }

    void update() override {
        int messagesLeft;
        int running;
        CURLMsg* msg;
        CURLMcode res = curl_multi_perform(multiHandle, &running);
        if (res != CURLM_OK) {
            auto message = curl_multi_strerror(res);
            logger.error() << message << " (" << url << ")";
            if (onReject) {
                onReject(HTTP_BAD_GATEWAY);
            }
            curl_multi_remove_handle(multiHandle, curl);
            url = "";
            return;
        }
        if ((msg = curl_multi_info_read(multiHandle, &messagesLeft)) != NULL) {
            if(msg->msg == CURLMSG_DONE) {
                curl_multi_remove_handle(multiHandle, curl);
            }
            int response;
            curl_easy_getinfo(msg->easy_handle, CURLINFO_RESPONSE_CODE, &response);
            if (response == HTTP_OK) {
                long size;
                if (!curl_easy_getinfo(curl, CURLINFO_REQUEST_SIZE, &size)) {
                    totalUpload += size;
                }
                if (!curl_easy_getinfo(curl, CURLINFO_HEADER_SIZE, &size)) {
                    totalDownload += size;
                }
                totalDownload += buffer.size();
                if (onResponse) {
                    onResponse(std::move(buffer));
                }
            } else {
                logger.error() << "response code " << response << " (" << url << ")";
                if (onReject) {
                    onReject(response);
                }
            }
            url = "";
        }
        if (url.empty() && !requests.empty()) {
            auto request = std::move(requests.front());
            requests.pop();
            processRequest(std::move(request));
        }
    }

    size_t getTotalUpload() const override {
        return totalUpload;
    }

    size_t getTotalDownload() const override {
        return totalDownload;
    }

    static std::unique_ptr<CurlRequests> create() {
        auto curl = curl_easy_init();
        if (curl == nullptr) {
            throw std::runtime_error("could not initialzie cURL");
        }
        auto multiHandle = curl_multi_init();
        if (multiHandle == nullptr) {
            curl_easy_cleanup(curl);
            throw std::runtime_error("could not initialzie cURL-multi");
        }
        return std::make_unique<CurlRequests>(multiHandle, curl);
    }
};

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

class SocketConnection : public Connection {
    SOCKET descriptor;
    sockaddr_in addr;
    size_t totalUpload = 0;
    size_t totalDownload = 0;
    ConnectionState state = ConnectionState::INITIAL;
    std::unique_ptr<std::thread> thread = nullptr;
    std::vector<char> readBatch;
    util::Buffer<char> buffer;
    std::mutex mutex;

    void connectSocket() {
        state = ConnectionState::CONNECTING;
        logger.info() << "connecting to " << to_string(addr);
        int res = connectsocket(descriptor, (const sockaddr*)&addr, sizeof(sockaddr_in));
        if (res < 0) {
            auto error = handle_socket_error("Connect failed");
            closesocket(descriptor);
            state = ConnectionState::CLOSED;
            logger.error() << error.what();
            return;
        }
        logger.info() << "connected to " << to_string(addr);
        state = ConnectionState::CONNECTED;
    }
public:
    SocketConnection(SOCKET descriptor, sockaddr_in addr)
        : descriptor(descriptor), addr(std::move(addr)), buffer(16'384) {}

    ~SocketConnection() {
        if (state != ConnectionState::CLOSED) {
            shutdown(descriptor, 2);
        }
        if (thread) {
            thread->join();
        }
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
            logger.debug() << "read " << size << " bytes from " << to_string(addr);
        }
    }

    void startClient() {
        state = ConnectionState::CONNECTED;
        thread = std::make_unique<std::thread>([this]() { startListen();});
    }

    void connect(runnable callback) override {
        thread = std::make_unique<std::thread>([this, callback]() {
            connectSocket();
            if (state == ConnectionState::CONNECTED) {
                callback();
            }
            startListen();
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
                shutdown(descriptor, 2);
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

    static std::shared_ptr<SocketConnection> connect(
        const std::string& address, int port, runnable callback
    ) {
        addrinfo hints {};

        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo* addrinfo = nullptr;
        if (int res = getaddrinfo(
            address.c_str(), nullptr, &hints, &addrinfo
        )) {
            throw std::runtime_error(gai_strerror(res));
        }

        sockaddr_in serverAddress;
        std::memcpy(&serverAddress, addrinfo->ai_addr, sizeof(sockaddr_in));
        serverAddress.sin_port = htons(port);
        freeaddrinfo(addrinfo);

        SOCKET descriptor = socket(AF_INET, SOCK_STREAM, 0);
        if (descriptor == -1) {
            throw std::runtime_error("Could not create socket");
        }
        auto socket = std::make_shared<SocketConnection>(descriptor, std::move(serverAddress));
        socket->connect(std::move(callback));
        return socket;
    }

    ConnectionState getState() const override {
        return state;
    }
};

class SocketTcpSServer : public TcpServer {
    u64id_t id;
    Network* network;
    SOCKET descriptor;
    std::vector<u64id_t> clients;
    std::mutex clientsMutex;
    bool open = true;
    std::unique_ptr<std::thread> thread = nullptr;
    int port;
public:
    SocketTcpSServer(u64id_t id, Network* network, SOCKET descriptor, int port)
    : id(id), network(network), descriptor(descriptor), port(port) {}

    ~SocketTcpSServer() {
        closeSocket();
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
                logger.info() << "client connected: " << to_string(address);
                auto socket = std::make_shared<SocketConnection>(
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
                if (auto client = network->getConnection(clientid)) {
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

    static std::shared_ptr<SocketTcpSServer> openServer(
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
#       ifndef _WIN32
            flags |= SO_REUSEPORT;
#       endif
        if (setsockopt(descriptor, SOL_SOCKET, flags, (const char*)&opt, sizeof(opt))) {
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
        logger.info() << "opened server at port " << port;
        auto server =
            std::make_shared<SocketTcpSServer>(id, network, descriptor, port);
        server->startListen(std::move(handler));
        return server;
    }
};

Network::Network(std::unique_ptr<Requests> requests)
: requests(std::move(requests)) {
}

Network::~Network() = default;

void Network::get(
    const std::string& url,
    OnResponse onResponse,
    OnReject onReject,
    long maxSize
) {
    requests->get(url, onResponse, onReject, maxSize);
}

void Network::post(
    const std::string& url,
    const std::string& fieldsData,
    OnResponse onResponse,
    OnReject onReject,
    long maxSize
) {
    requests->post(url, fieldsData, onResponse, onReject, maxSize);
}

Connection* Network::getConnection(u64id_t id) {
    std::lock_guard lock(connectionsMutex);

    const auto& found = connections.find(id);
    if (found == connections.end()) {
        return nullptr;
    }
    return found->second.get();
}

TcpServer* Network::getServer(u64id_t id) const {
    const auto& found = servers.find(id);
    if (found == servers.end()) {
        return nullptr;
    }
    return found->second.get();
}

u64id_t Network::connect(const std::string& address, int port, consumer<u64id_t> callback) {
    std::lock_guard lock(connectionsMutex);
    
    u64id_t id = nextConnection++;
    auto socket = SocketConnection::connect(address, port, [id, callback]() {
        callback(id);
    });
    connections[id] = std::move(socket);
    return id;
}

u64id_t Network::openServer(int port, ConnectCallback handler) {
    u64id_t id = nextServer++;
    auto server = SocketTcpSServer::openServer(id, this, port, handler);
    servers[id] = std::move(server);
    return id;
}

u64id_t Network::addConnection(const std::shared_ptr<Connection>& socket) {
    std::lock_guard lock(connectionsMutex);

    u64id_t id = nextConnection++;
    connections[id] = std::move(socket);
    return id;
}

size_t Network::getTotalUpload() const {
    return requests->getTotalUpload() + totalUpload;
}

size_t Network::getTotalDownload() const {
    return requests->getTotalDownload() + totalDownload;
}

void Network::update() {
    requests->update();

    {
        std::lock_guard lock(connectionsMutex);
        auto socketiter = connections.begin();
        while (socketiter != connections.end()) {
            auto socket = socketiter->second.get();
            totalDownload += socket->pullDownload();
            totalUpload += socket->pullUpload();
            if (socket->available() == 0 && 
                socket->getState() == ConnectionState::CLOSED) {
                socketiter = connections.erase(socketiter);
                continue;
            }
            ++socketiter;
        }
        auto serveriter = servers.begin();
        while (serveriter != servers.end()) {
            auto server = serveriter->second.get();
            if (!server->isOpen()) {
                serveriter = servers.erase(serveriter);
                continue;
            }
            ++serveriter;
        }
    }
}

std::unique_ptr<Network> Network::create(const NetworkSettings& settings) {
    auto requests = CurlRequests::create();
    return std::make_unique<Network>(std::move(requests));
}
