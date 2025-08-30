#pragma once

#include <memory>
#include <vector>
#include <mutex>

#include "typedefs.hpp"
#include "settings.hpp"
#include "util/Buffer.hpp"
#include "delegates.hpp"

namespace network {
    using OnResponse = std::function<void(std::vector<char>)>;
    using OnReject = std::function<void(int)>;
    using ConnectCallback = std::function<void(u64id_t, u64id_t)>;

    class Requests {
    public:
        virtual ~Requests() {}

        virtual void get(
            const std::string& url,
            OnResponse onResponse,
            OnReject onReject=nullptr,
            long maxSize=0
        ) = 0;

        virtual void post(
            const std::string& url,
            const std::string& data,
            OnResponse onResponse,
            OnReject onReject=nullptr,
            long maxSize=0
        ) = 0;

        virtual size_t getTotalUpload() const = 0;
        virtual size_t getTotalDownload() const = 0;

        virtual void update() = 0;
    };

    enum class ConnectionState {
        INITIAL, CONNECTING, CONNECTED, CLOSED
    };

    class Connection {
    public:
        virtual ~Connection() {}

        virtual void connect(runnable callback) = 0;
        virtual int recv(char* buffer, size_t length) = 0;
        virtual int send(const char* buffer, size_t length) = 0;
        virtual void close(bool discardAll=false) = 0;
        virtual int available() = 0;

        virtual size_t pullUpload() = 0;
        virtual size_t pullDownload() = 0;

        virtual int getPort() const = 0;
        virtual std::string getAddress() const = 0;

        virtual ConnectionState getState() const = 0;
    };

    class TcpServer {
    public:
        virtual ~TcpServer() {}
        virtual void startListen(ConnectCallback handler) = 0;
        virtual void close() = 0;
        virtual bool isOpen() = 0;
        virtual int getPort() const = 0;
    };

    class Network {
        std::unique_ptr<Requests> requests;

        std::unordered_map<u64id_t, std::shared_ptr<Connection>> connections;
        std::mutex connectionsMutex {};
        u64id_t nextConnection = 1;

        std::unordered_map<u64id_t, std::shared_ptr<TcpServer>> servers;
        u64id_t nextServer = 1;

        size_t totalDownload = 0;
        size_t totalUpload = 0;
    public:
        Network(std::unique_ptr<Requests> requests);
        ~Network();

        void get(
            const std::string& url,
            OnResponse onResponse,
            OnReject onReject = nullptr,
            long maxSize=0
        );

        void post(
            const std::string& url,
            const std::string& fieldsData,
            OnResponse onResponse,
            OnReject onReject = nullptr,
            long maxSize=0
        );

        [[nodiscard]] Connection* getConnection(u64id_t id);
        [[nodiscard]] TcpServer* getServer(u64id_t id) const;

        u64id_t connect(const std::string& address, int port, consumer<u64id_t> callback);

        u64id_t openServer(int port, ConnectCallback handler);

        u64id_t addConnection(const std::shared_ptr<Connection>& connection);

        size_t getTotalUpload() const;
        size_t getTotalDownload() const;

        void update();

        static std::unique_ptr<Network> create(const NetworkSettings& settings);
    };
}
