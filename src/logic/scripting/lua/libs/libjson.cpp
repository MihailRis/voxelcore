#include "coders/json.hpp"
#include "api_lua.hpp"

static int l_json_stringify(lua::State* L) {
    auto value = lua::tovalue(L, 1);

    bool nice = lua::toboolean(L, 2);
    bool escapeUTF = lua::toboolean(L, 3);
    auto string = json::stringify(value, nice, "  ", escapeUTF);
    return lua::pushstring(L, string);
}

static int l_json_parse(lua::State* L) {
    auto string = lua::require_string(L, 1);
    auto element = json::parse("[string]", string);
    return lua::pushvalue(L, element);
}

const luaL_Reg jsonlib[] = {
    {"tostring", lua::wrap<l_json_stringify>},
    {"parse", lua::wrap<l_json_parse>},
    {nullptr, nullptr}
};
