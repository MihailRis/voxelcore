#include "DebuggingServer.hpp"

#include "engine/Engine.hpp"
#include "network/Network.hpp"
#include "debug/Logger.hpp"
#include "coders/json.hpp"

using namespace devtools;

static debug::Logger logger("debug-server");

ClientConnection::~ClientConnection() {
    if (auto connection = dynamic_cast<network::ReadableConnection*>(
        network.getConnection(this->connection, true)
    )) {
        connection->close();
    }
}

bool ClientConnection::initiate(network::ReadableConnection* connection) {
    if (connection->available() < 8) {
        return false;
    }
    char buffer[8] {};
    char expected[8] {};
    std::memcpy(expected, VCDBG_MAGIC, sizeof(VCDBG_MAGIC));
    expected[6] = VCDBG_VERSION >> 8;
    expected[7] = VCDBG_VERSION & 0xFF;
    connection->recv(buffer, sizeof(VCDBG_MAGIC));

    connection->send(expected, sizeof(VCDBG_MAGIC));
    if (std::memcmp(expected, buffer, sizeof(VCDBG_MAGIC)) == 0) {
        initiated = true;
        return false;
    } else {
        connection->close(true);
        return true;
    }
}

std::string ClientConnection::read() {
    auto connection = dynamic_cast<network::ReadableConnection*>(
        network.getConnection(this->connection, true)
    );
    if (connection == nullptr) {
        return "";
    }
    if (!initiated) {
        if (initiate(connection)) {
            return "";
        }
    }
    if (messageLength == 0) {
        if (connection->available() >= sizeof(int32_t)) {
            int32_t length = 0;
            connection->recv(reinterpret_cast<char*>(&length), sizeof(int32_t));
            if (length <= 0) {
                logger.error() << "invalid message length " << length;
            } else {
                logger.info() << "message length " << length;
                messageLength = length;
            }
        }
    } else if (connection->available() >= messageLength) {
        std::string string(messageLength, 0);
        connection->recv(string.data(), messageLength);
        messageLength = 0;
        return string;
    }
    return "";
}

void ClientConnection::send(const dv::value& object) {
    auto connection = dynamic_cast<network::ReadableConnection*>(
        network.getConnection(this->connection, true)
    );
    if (connection == nullptr) {
        return;
    }
    auto message = json::stringify(object, false);
    int32_t length = message.length();
    connection->send(reinterpret_cast<char*>(&length), sizeof(int32_t));
    connection->send(message.data(), length);
}

void ClientConnection::sendResponse(const std::string& type) {
    send(dv::object({{"type", type}}));
}

bool ClientConnection::alive() const {
    return network.getConnection(this->connection, true) != nullptr;
}

static network::Server& create_tcp_server(
    DebuggingServer& dbgServer, Engine& engine, int port
) {
    auto& network = engine.getNetwork();
    u64id_t serverId = network.openTcpServer(
        port,
        [&network, &dbgServer](u64id_t sid, u64id_t id) {
            auto& connection = dynamic_cast<network::ReadableConnection&>(
                *network.getConnection(id, true)
            );
            connection.setPrivate(true);
            logger.info() << "connected client " << id << ": "
                          << connection.getAddress() << ":"
                          << connection.getPort();
            dbgServer.setClient(id);
        }
    );
    auto& server = *network.getServer(serverId, true);
    server.setPrivate(true);

    auto& tcpServer = dynamic_cast<network::TcpServer&>(server);
    tcpServer.setMaxClientsConnected(1);

    logger.info() << "tcp debugging server open at port " << server.getPort();

    return tcpServer;
}

static network::Server& create_server(
    DebuggingServer& dbgServer, Engine& engine, const std::string& serverString
) {
    logger.info() << "starting debugging server";

    size_t sepPos = serverString.find(':');
    if (sepPos == std::string::npos) {
        throw std::runtime_error("invalid debugging server configuration string");
    }
    auto transport = serverString.substr(0, sepPos);
    if (transport == "tcp") {
        int port;
        try {
            port = std::stoi(serverString.substr(sepPos + 1));
        } catch (const std::exception& err) {
            throw std::runtime_error("invalid tcp port");
        }
        return create_tcp_server(dbgServer, engine, port);
    } else {
        throw std::runtime_error(
            "unsupported debugging server transport '" + transport + "'"
        );
    }
}

DebuggingServer::DebuggingServer(
    Engine& engine, const std::string& serverString
)
    : engine(engine),
      server(create_server(*this, engine, serverString)),
      connection(nullptr) {
}

DebuggingServer::~DebuggingServer() {
    logger.info() << "stopping debugging server";
    server.close();
}


bool DebuggingServer::update() {
    if (connection == nullptr) {
        return false;
    }
    std::string message = connection->read();
    if (message.empty()) {
        if (!connection->alive()) {
            bool status = performCommand(disconnectAction, dv::object());
            connection.reset();
            return status;
        }
        return false;
    }
    logger.debug() << "received: " << message;
    try {
        auto obj = json::parse(message);
        if (!obj.has("type")) {
            logger.error() << "missing message type";
            return false;
        }
        const auto& type = obj["type"].asString();
        if (performCommand(type, obj)) {
            connection->sendResponse("resumed");
            return true;
        }
    } catch (const std::runtime_error& err) {
        logger.error() << "could not to parse message: " << err.what();
    }
    return false;
}

bool DebuggingServer::performCommand(
    const std::string& type, const dv::value& map
) {
    if (!connectionEstablished && type == "connect") {
        map.at("disconnect-action").get(disconnectAction);
        connectionEstablished = true;
        logger.info() << "client connection established";
        connection->sendResponse("success");
        return true;
    }
    if (!connectionEstablished) {
        return false;
    }
    if (type == "terminate") {
        engine.quit();
        connection->sendResponse("success");
    } else if (type == "detach") {
        connection->sendResponse("success");
        connection.reset();
        engine.detachDebugger();
        return false;
    } else if (type == "set-breakpoint" || type == "remove-breakpoint") {
        if (!map.has("source") || !map.has("line"))
            return false;
        breakpointEvents.push_back(DebuggingEvent {
            type[0] == 's' 
            ? DebuggingEventType::SET_BREAKPOINT
            : DebuggingEventType::REMOVE_BREAKPOINT,
            BreakpointEventDto {
                map["source"].asString(),
                static_cast<int>(map["line"].asInteger()),
            }
        });
    } else if (type == "step" || type == "step-into-function") {
        breakpointEvents.push_back(DebuggingEvent {
            type == "step"
            ? DebuggingEventType::STEP
            : DebuggingEventType::STEP_INTO_FUNCTION,
            SignalEventDto {}
        });
        return true;
    } else if (type == "resume") {
        breakpointEvents.push_back(DebuggingEvent {
            DebuggingEventType::RESUME, SignalEventDto {}});
        return true;
    } else if (type == "get-value") {
        if (!map.has("frame") || !map.has("local") || !map.has("path"))
            return false;

        int frame = map["frame"].asInteger();
        int localIndex = map["local"].asInteger();

        ValuePath path;
        for (const auto& segment : map["path"]) {
            if (segment.isString()) {
                path.emplace_back(segment.asString());
            } else {
                path.emplace_back(static_cast<int>(segment.asInteger()));
            }
        }
        breakpointEvents.push_back(DebuggingEvent {
            DebuggingEventType::GET_VALUE, GetValueEventDto {
                frame, localIndex, std::move(path)
            }
        });
        return true;
    } else {
        logger.error() << "unsupported command '" << type << "'";
    }
    return false;
}

void DebuggingServer::pause(
    std::string&& reason, std::string&& message, dv::value&& stackTrace
) {
    if (connection == nullptr) {
        return;
    }
    auto response = dv::object({{"type", std::string("paused")}});
    if (!reason.empty()) {
        response["reason"] = std::move(reason);
    }
    if (!message.empty()) {
        response["message"] = std::move(message);
    }
    if (stackTrace != nullptr) {
        response["stack"] = std::move(stackTrace);
    }
    connection->send(std::move(response));
    engine.startPauseLoop();
}

void DebuggingServer::sendValue(
    dv::value&& value, int frame, int local, ValuePath&& path
) {
    auto pathValue = dv::list();
    for (const auto& segment : path) {
        if (auto string = std::get_if<std::string>(&segment)) {
            pathValue.add(*string);
        } else {
            pathValue.add(std::get<int>(segment));
        }
    }
    connection->send(dv::object({
        {"type", std::string("value")},
        {"frame", frame},
        {"local", local},
        {"path", std::move(pathValue)},
        {"value", std::move(value)},
    }));
}

void DebuggingServer::setClient(u64id_t client) {
    connection =
        std::make_unique<ClientConnection>(engine.getNetwork(), client);
    connectionEstablished = false;
}

std::vector<DebuggingEvent> DebuggingServer::pullEvents() {
    return std::move(breakpointEvents);
}

void DebuggingServer::setDisconnectAction(const std::string& action) {
    disconnectAction = action;
}
