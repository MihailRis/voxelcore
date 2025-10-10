#include "../lua_custom_types.hpp"
#include "../lua_util.hpp"

#include <chrono>

using namespace lua;
using namespace std::chrono;

static int l_random(lua::State* L) {
    std::uniform_int_distribution<> dist(0, std::numeric_limits<int>::max());

    auto& rng = require_userdata<LuaRandom>(L, 1).rng;
    size_t n = touinteger(L, 2);
    createtable(L, n, 0);

    for (size_t i = 0; i < n; i++) {
        pushnumber(L, dist(rng) / (double)std::numeric_limits<int>::max());
        rawseti(L, i + 1);
    }
    return 1;
}

static int l_seed(lua::State* L) {
    require_userdata<LuaRandom>(L, 1).rng = std::mt19937(lua::touinteger(L, 2));
    return 0;
}

static int l_meta_meta_call(lua::State* L) {
    integer_t seed;
    if (lua::isnoneornil(L, 1)) {
        seed = system_clock::now().time_since_epoch().count();
    } else {
        seed = tointeger(L, 1);
    }
    return newuserdata<LuaRandom>(L, seed);
}

int LuaRandom::createMetatable(lua::State* L) {
    createtable(L, 0, 3);

    requireglobal(L, "__vc_create_random_methods");
    createtable(L, 0, 0);
    pushcfunction(L, wrap<l_random>);
    setfield(L, "random");
    pushcfunction(L, wrap<l_seed>);
    setfield(L, "seed");
    call(L, 1, 1);

    setfield(L, "__index");

    createtable(L, 0, 1);
    pushcfunction(L, wrap<l_meta_meta_call>);
    setfield(L, "__call");
    setmetatable(L);
    return 1;
}
