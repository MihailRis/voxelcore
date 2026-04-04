#pragma once

#include "commons.hpp"

namespace network {
    class TcpConnection : public ReadableConnection {
    public:
        ~TcpConnection() override = default;

        virtual void connect(runnable callback, stringconsumer errorCallback) = 0;

        virtual void setNoDelay(bool noDelay) = 0;
        [[nodiscard]] virtual bool isNoDelay() const = 0;

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

    class TcpServer : public Server {
    public:
        ~TcpServer() override {}
        virtual void startListen(ConnectCallback handler) = 0;

        [[nodiscard]] TransportType getTransportType() const noexcept override {
            return TransportType::TCP;
        }

        virtual void setMaxClientsConnected(int count) = 0;
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

        [[nodiscard]] Connection* getConnection(u64id_t id, bool includePrivate);
        [[nodiscard]] Server* getServer(u64id_t id, bool includePrivate) const;

        int findFreePort() const;

        u64id_t connectTcp(
            const std::string& address,
            int port,
            consumer<u64id_t> callback,
            ConnectErrorCallback errorCallback
        );
        u64id_t connectUdp(
            const std::string& address,
            int port,
            const consumer<u64id_t>& callback,
            ClientDatagramCallback handler
        );

        u64id_t openTcpServer(int port, ConnectCallback handler);
        u64id_t openUdpServer(int port, const ServerDatagramCallback& handler);

        u64id_t addConnection(const std::shared_ptr<Connection>& connection);

        [[nodiscard]] size_t getTotalUpload() const;
        [[nodiscard]] size_t getTotalDownload() const;

        void update();

        static std::unique_ptr<Network> create(const NetworkSettings& settings);
    };
}
