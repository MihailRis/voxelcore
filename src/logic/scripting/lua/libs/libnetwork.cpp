#include "api_lua.hpp"

#include "engine/Engine.hpp"
#include "network/Network.hpp"
#include "coders/json.hpp"

using namespace scripting;

static int l_get(lua::State* L, network::Network& network) {
    std::string url(lua::require_lstring(L, 1));

    lua::pushvalue(L, 2);
    auto onResponse = lua::create_lambda_nothrow(L);

    network.get(url, [onResponse](std::vector<char> bytes) {
        engine->postRunnable([=]() {
            onResponse({std::string(bytes.data(), bytes.size())});
        });
    });
    return 0;
}

static int l_get_binary(lua::State* L, network::Network& network) {
    std::string url(lua::require_lstring(L, 1));

    lua::pushvalue(L, 2);
    auto onResponse = lua::create_lambda_nothrow(L);

    network.get(url, [onResponse](std::vector<char> bytes) {
        auto buffer = std::make_shared<util::Buffer<ubyte>>(
            reinterpret_cast<const ubyte*>(bytes.data()), bytes.size()
        );
        engine->postRunnable([=]() {
            onResponse({buffer});
        });
    });
    return 0;
}

static int l_post(lua::State* L, network::Network& network) {
    std::string url(lua::require_lstring(L, 1));
    auto data = lua::tovalue(L, 2);

    lua::pushvalue(L, 3);
    auto onResponse = lua::create_lambda_nothrow(L);

    std::string string;
    if (data.isString()) {
        string = data.asString();
    } else {
        string = json::stringify(data, false);
    }
    engine->getNetwork().post(url, string, [onResponse](std::vector<char> bytes) {
        auto buffer = std::make_shared<util::Buffer<ubyte>>(
            reinterpret_cast<const ubyte*>(bytes.data()), bytes.size()
        );
        engine->postRunnable([=]() {
            onResponse({std::string(bytes.data(), bytes.size())});
        });
    });
    return 0;
}

static int l_connect(lua::State* L, network::Network& network) {
    std::string address = lua::require_string(L, 1);
    int port = lua::tointeger(L, 2);
    lua::pushvalue(L, 3);
    auto callback = lua::create_lambda_nothrow(L);
    u64id_t id = network.connect(address, port, [callback](u64id_t id) {
        engine->postRunnable([=]() {
            callback({id});
        });
    });
    return lua::pushinteger(L, id);
}


static int l_close(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto connection = network.getConnection(id)) {
        connection->close(true);
    }
    return 0;
}

static int l_closeserver(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto server = network.getServer(id)) {
        server->close();
    }
    return 0;
}

static int l_send(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    auto connection = network.getConnection(id);
    if (connection == nullptr ||
        connection->getState() == network::ConnectionState::CLOSED) {
        return 0;
    }
    if (lua::istable(L, 2)) {
        lua::pushvalue(L, 2);
        size_t size = lua::objlen(L, 2);
        util::Buffer<char> buffer(size);
        for (size_t i = 0; i < size; i++) {
            lua::rawgeti(L, i + 1);
            buffer[i] = lua::tointeger(L, -1);
            lua::pop(L);
        }
        lua::pop(L);
        connection->send(buffer.data(), size);
    } else if (auto bytes = lua::touserdata<lua::LuaBytearray>(L, 2)) {
        connection->send(
            reinterpret_cast<char*>(bytes->data().data()), bytes->data().size()
        );
    } else if (lua::isstring(L, 2)) {
        auto string = lua::tolstring(L, 2);
        connection->send(string.data(), string.length());
    }
    return 0;
}

static int l_recv(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    int length = lua::tointeger(L, 2);
    auto connection = engine->getNetwork().getConnection(id);
    if (connection == nullptr) {
        return 0;
    }
    length = glm::min(length, connection->available());
    util::Buffer<char> buffer(length);
    
    int size = connection->recv(buffer.data(), length);
    if (size == -1) {
        return 0;
    }
    if (lua::toboolean(L, 3)) {
        lua::createtable(L, size, 0);
        for (size_t i = 0; i < size; i++) {
            lua::pushinteger(L, buffer[i] & 0xFF);
            lua::rawseti(L, i+1);
        }
    } else {
        lua::newuserdata<lua::LuaBytearray>(L, size);
        auto bytearray = lua::touserdata<lua::LuaBytearray>(L, -1);   
        bytearray->data().reserve(size);
        std::memcpy(bytearray->data().data(), buffer.data(), size);
    }
    return 1;
}

static int l_available(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto connection = network.getConnection(id)) {
        return lua::pushinteger(L, connection->available());
    }
    return 0;
}

static int l_open(lua::State* L, network::Network& network) {
    int port = lua::tointeger(L, 1);
    lua::pushvalue(L, 2);
    auto callback = lua::create_lambda_nothrow(L);
    u64id_t id = network.openServer(port, [callback](u64id_t id) {
        engine->postRunnable([=]() {
            callback({id});
        });
    });
    return lua::pushinteger(L, id);
}

static int l_is_alive(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto connection = network.getConnection(id)) {
        return lua::pushboolean(
            L,
            connection->getState() != network::ConnectionState::CLOSED ||
                connection->available() > 0
        );
    }
    return lua::pushboolean(L, false);
}

static int l_is_connected(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto connection = network.getConnection(id)) {
        return lua::pushboolean(
            L, connection->getState() == network::ConnectionState::CONNECTED
        );
    }
    return lua::pushboolean(L, false);
}

static int l_get_address(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto connection = network.getConnection(id)) {
        lua::pushstring(L, connection->getAddress());
        lua::pushinteger(L, connection->getPort());
        return 2;
    }
    return 0;
}

static int l_is_serveropen(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto server = network.getServer(id)) {
        return lua::pushboolean(L, server->isOpen());
    }
    return lua::pushboolean(L, false);
}

static int l_get_serverport(lua::State* L, network::Network& network) {
    u64id_t id = lua::tointeger(L, 1);
    if (auto server = network.getServer(id)) {
        return lua::pushinteger(L, server->getPort());
    }
    return 0;
}

static int l_get_total_upload(lua::State* L, network::Network& network) {
    return lua::pushinteger(L, network.getTotalUpload());
}

static int l_get_total_download(lua::State* L, network::Network& network) {
    return lua::pushinteger(L, network.getTotalDownload());
}

template <int(*func)(lua::State*, network::Network&)>
int wrap(lua_State* L) {
    int result = 0;
    try {
        result = func(L, engine->getNetwork());
    }
    // transform exception with description into lua_error
    catch (std::exception& e) {
        luaL_error(L, e.what());
    }
    // Rethrow any other exception (lua error for example)
    catch (...) {
        throw;
    }
    return result;
}


const luaL_Reg networklib[] = {
    {"get", wrap<l_get>},
    {"get_binary", wrap<l_get_binary>},
    {"post", wrap<l_post>},
    {"get_total_upload", wrap<l_get_total_upload>},
    {"get_total_download", wrap<l_get_total_download>},
    {"__open", wrap<l_open>},
    {"__closeserver", wrap<l_closeserver>},
    {"__connect", wrap<l_connect>},
    {"__close", wrap<l_close>},
    {"__send", wrap<l_send>},
    {"__recv", wrap<l_recv>},
    {"__available", wrap<l_available>},
    {"__is_alive", wrap<l_is_alive>},
    {"__is_connected", wrap<l_is_connected>},
    {"__get_address", wrap<l_get_address>},
    {"__is_serveropen", wrap<l_is_serveropen>},
    {"__get_serverport", wrap<l_get_serverport>},
    {NULL, NULL}
};
