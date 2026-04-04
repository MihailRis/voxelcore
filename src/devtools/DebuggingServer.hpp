#pragma once

#include <memory>
#include <string>
#include <vector>
#include <variant>

#include "typedefs.hpp"

namespace network {
    class Server;
    class Connection;
    class ReadableConnection;
    class Network;
}

namespace dv {
    class value;
}

class Engine;

namespace devtools {
    inline constexpr const char VCDBG_MAGIC[8] = "vc-dbg\0";
    inline constexpr int VCDBG_VERSION = 1;

    class ClientConnection {
    public:
        ClientConnection(network::Network& network, u64id_t connection)
            : network(network), connection(connection) {
        }
        ~ClientConnection();

        std::string read();
        void send(const dv::value& message);
        void sendResponse(const std::string& type);

        bool alive() const;
    private:
        network::Network& network;
        size_t messageLength = 0;
        u64id_t connection;
        bool initiated = false;
    
        bool initiate(network::ReadableConnection* connection);
    };

    enum class DebuggingEventType {
        SET_BREAKPOINT = 1,
        REMOVE_BREAKPOINT,
        STEP,
        STEP_INTO_FUNCTION,
        RESUME,
        GET_VALUE,
    };

    struct BreakpointEventDto {
        std::string source;
        int line;
    };

    struct SignalEventDto {
    };

    using ValuePath = std::vector<std::variant<std::string, int>>;

    struct GetValueEventDto {
        int frame;
        int localIndex;
        ValuePath path;
    };

    struct DebuggingEvent {
        DebuggingEventType type;
        std::variant<BreakpointEventDto, SignalEventDto, GetValueEventDto> data;
    };

    class DebuggingServer {
    public:
        DebuggingServer(Engine& engine, const std::string& serverString);
        ~DebuggingServer();

        bool update();
        void pause(
            std::string&& reason, std::string&& message, dv::value&& stackTrace
        );

        void sendValue(dv::value&& value, int frame, int local, ValuePath&& path);

        void setClient(u64id_t client);
        std::vector<DebuggingEvent> pullEvents();

        void setDisconnectAction(const std::string& action);
    private:
        Engine& engine;
        network::Server& server;
        std::unique_ptr<ClientConnection> connection;
        bool connectionEstablished = false;
        std::vector<DebuggingEvent> breakpointEvents;
        std::string disconnectAction = "resume";

        bool performCommand(
            const std::string& type, const dv::value& map
        );
    };
}
