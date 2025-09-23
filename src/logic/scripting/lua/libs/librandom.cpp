#include "api_lua.hpp"

#include "util/random.hpp"

static std::random_device random_device;

static int l_random(lua::State* L) {
    int argc = lua::gettop(L);

    auto randomEngine = util::seeded_random_engine(random_device);
    if (argc == 0) {
        std::uniform_real_distribution<> dist(0.0, 1.0);
        return lua::pushnumber(L, dist(randomEngine));
    } else if (argc == 1) {
        std::uniform_int_distribution<integer_t> dist(1, lua::tointeger(L, 1));
        return lua::pushinteger(L, dist(randomEngine));
    } else {
        std::uniform_int_distribution<integer_t> dist(
            lua::tointeger(L, 1), lua::tointeger(L, 2)
        );
        return lua::pushinteger(L, dist(randomEngine));
    }
}

static int l_bytes(lua::State* L) {
    size_t size = lua::tointeger(L, 1);

    auto randomEngine = util::seeded_random_engine(random_device);
    static std::uniform_int_distribution<integer_t> dist(0, 0xFF);
    std::vector<ubyte> bytes (size);
    for (size_t i = 0; i < bytes.size(); i++) {
        bytes[i] = dist(randomEngine);
    }
    return lua::create_bytearray(L, bytes);
}

static int l_uuid(lua::State* L) {
    return lua::pushlstring(L, util::generate_uuid());
}

const luaL_Reg randomlib[] = {
    {"random", lua::wrap<l_random>},
    {"bytes", lua::wrap<l_bytes>},
    {"uuid", lua::wrap<l_uuid>},
    {NULL, NULL}
};
