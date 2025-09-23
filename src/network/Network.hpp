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
    using OnReject = std::function<void(int, std::vector<char>)>;
    using ConnectCallback = std::function<void(u64id_t, u64id_t)>;
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

        [[nodiscard]] virtual int getPort() const = 0;
        [[nodiscard]] virtual std::string getAddress() const = 0;

        [[nodiscard]] virtual ConnectionState getState() const = 0;

        [[nodiscard]] virtual TransportType getTransportType() const noexcept = 0;
    };

    class TcpConnection : public Connection {
    public:
        ~TcpConnection() override = default;

        virtual void connect(runnable callback) = 0;
        virtual int recv(char* buffer, size_t length) = 0;
        virtual int available() = 0;

        [[nodiscard]] TransportType getTransportType() const noexcept override {
            return TransportType::TCP;
        }
    };

    class UdpConnection : public Connection {
    public:
        ~UdpConnection() override = default;

        virtual void connect(ClientDatagramCallback handler) = 0;

        [[nodiscard]] TransportType getTransportType() const noexcept override {
            return TransportType::UDP;
        }
    };

    class Server {
    public:
        virtual ~Server() = default;
        virtual void close() = 0;
        virtual bool isOpen() = 0;
        [[nodiscard]] virtual TransportType getTransportType() const noexcept = 0;
        [[nodiscard]] virtual int getPort() const = 0;
    };

    class TcpServer : public Server {
    public:
        ~TcpServer() override {}
        virtual void startListen(ConnectCallback handler) = 0;

        [[nodiscard]] TransportType getTransportType() const noexcept override {
            return TransportType::TCP;
        }
    };

    class UdpServer : public Server {
    public:
        ~UdpServer() override {}
        virtual void startListen(ServerDatagramCallback handler) = 0;

        virtual void sendTo(const std::string& addr, int port, const char* buffer, size_t length) = 0;

        [[nodiscard]] TransportType getTransportType() const noexcept override {
            return TransportType::UDP;
        }
    };

    class Network {
        std::unique_ptr<Requests> requests;

        std::unordered_map<u64id_t, std::shared_ptr<Connection>> connections;
        std::mutex connectionsMutex {};
        u64id_t nextConnection = 1;

        std::unordered_map<u64id_t, std::shared_ptr<Server>> servers;
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
            std::vector<std::string> headers = {},
            long maxSize=0
        );

        void post(
            const std::string& url,
            const std::string& fieldsData,
            OnResponse onResponse,
            OnReject onReject = nullptr,
            std::vector<std::string> headers = {},
            long maxSize=0
        );

        [[nodiscard]] Connection* getConnection(u64id_t id);
        [[nodiscard]] Server* getServer(u64id_t id) const;

        u64id_t connectTcp(const std::string& address, int port, consumer<u64id_t> callback);
        u64id_t connectUdp(const std::string& address, int port, const consumer<u64id_t>& callback, ClientDatagramCallback handler);

        u64id_t openTcpServer(int port, ConnectCallback handler);
        u64id_t openUdpServer(int port, const ServerDatagramCallback& handler);

        u64id_t addConnection(const std::shared_ptr<Connection>& connection);

        [[nodiscard]] size_t getTotalUpload() const;
        [[nodiscard]] size_t getTotalDownload() const;

        void update();

        static std::unique_ptr<Network> create(const NetworkSettings& settings);
    };
}
