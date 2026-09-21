#pragma once

#include "typedefs.hpp"
#include "settings.hpp"
#include "util/Buffer.hpp"
#include "delegates.hpp"

#include <memory>
#include <vector>
#include <mutex>

namespace network {
    struct HttpResponse;
    struct HttpServerRequest;

    using OnResponse = std::function<void(HttpResponse)>;
    using ConnectCallback = std::function<void(u64id_t, u64id_t)>;
    using ConnectErrorCallback = std::function<void(u64id_t, std::string)>;
    using ServerDatagramCallback = std::function<void(u64id_t sid, const std::string& addr, int port, const char* buffer, size_t length)>;
    using ClientDatagramCallback = std::function<void(u64id_t cid, const char* buffer, size_t length)>;
    using HttpRequestCallback = std::function<void(u64id_t sid, HttpServerRequest)>;

    struct HttpRequest {
        std::string method;
        std::string url;
        std::string body;
        std::vector<std::string> headers;

        OnResponse onResponse;
        bool followLocation = false;
        bool verifySSL = true;
        long maxSize = -1;
        long timeoutMs = 0;
    };

    struct HttpResponse {
        int status;
        std::vector<std::string> headers;
        std::vector<char> body;
    };

    struct HttpServerRequest {
        u64id_t requestId;
        std::string method;
        std::string path;
        std::string query;
        std::vector<std::string> headers;
        std::string body;
        std::string remoteAddr;
        int remotePort = 0;
    };

    struct HttpServerResponse {
        int status = 200;
        std::vector<std::string> headers;
        std::string body;
    };

    class Requests {
    public:
        virtual ~Requests() {}

        virtual void request(HttpRequest request) = 0;

        [[nodiscard]] virtual size_t getTotalUpload() const = 0;
        [[nodiscard]] virtual size_t getTotalDownload() const = 0;

        virtual void update() = 0;
    };

    enum class ConnectionState {
        INITIAL, CONNECTING, CONNECTED, CLOSED
    };

    enum class TransportType {
        TCP, UDP, HTTP
    };

    class Connection {
    public:
        virtual ~Connection() = default;

        virtual void close(bool discardAll=false) = 0;

        virtual int send(const char* buffer, size_t length) = 0;

        virtual size_t pullUpload() = 0;
        virtual size_t pullDownload() = 0;

        bool isPrivate() const { return isprivate; }
        void setPrivate(bool flag) {isprivate = flag;}

        [[nodiscard]] virtual int getPort() const = 0;
        [[nodiscard]] virtual std::string getAddress() const = 0;

        [[nodiscard]] virtual ConnectionState getState() const = 0;

        [[nodiscard]] virtual TransportType getTransportType() const noexcept = 0;
    protected:
        bool isprivate = false;
    };

    class ReadableConnection : public Connection {
    public:
        virtual int recv(char* buffer, size_t length) = 0;
        virtual int peek(char* buffer, size_t length) = 0;
        virtual int available() = 0;
    };

    class Server {
    public:
        virtual ~Server() = default;

        virtual void update() = 0;
        virtual void close() = 0;
        virtual bool isOpen() = 0;
        [[nodiscard]] virtual TransportType getTransportType() const noexcept = 0;
        [[nodiscard]] virtual int getPort() const = 0;

        bool isPrivate() const { return isprivate; }
        void setPrivate(bool flag) { isprivate = flag; }
    protected:
        bool isprivate = false;
    };
}
