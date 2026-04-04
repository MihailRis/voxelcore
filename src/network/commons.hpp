#pragma once

#include "typedefs.hpp"
#include "settings.hpp"
#include "util/Buffer.hpp"
#include "delegates.hpp"

#include <memory>
#include <vector>
#include <mutex>

namespace network {
    using OnResponse = std::function<void(std::vector<char>)>;
    using OnReject = std::function<void(int, std::vector<char>)>;
    using ConnectCallback = std::function<void(u64id_t, u64id_t)>;
    using ConnectErrorCallback = std::function<void(u64id_t, std::string)>;
    using ServerDatagramCallback = std::function<void(u64id_t sid, const std::string& addr, int port, const char* buffer, size_t length)>;
    using ClientDatagramCallback = std::function<void(u64id_t cid, const char* buffer, size_t length)>;

    class Requests {
    public:
        virtual ~Requests() {}

        virtual void get(
            const std::string& url,
            OnResponse onResponse,
            OnReject onReject=nullptr,
            std::vector<std::string> headers = {},
            long maxSize=0
        ) = 0;

        virtual void post(
            const std::string& url,
            const std::string& data,
            OnResponse onResponse,
            OnReject onReject=nullptr,
            std::vector<std::string> headers = {},
            long maxSize=0
        ) = 0;

        [[nodiscard]] virtual size_t getTotalUpload() const = 0;
        [[nodiscard]] virtual size_t getTotalDownload() const = 0;

        virtual void update() = 0;
    };

    enum class ConnectionState {
        INITIAL, CONNECTING, CONNECTED, CLOSED
    };

    enum class TransportType {
        TCP, UDP
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
