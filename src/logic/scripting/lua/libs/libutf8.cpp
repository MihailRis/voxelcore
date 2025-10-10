#include "api_lua.hpp"

#include <vector>
#include <cwctype>

#include "../lua_custom_types.hpp"
#include "util/stringutil.hpp"

static int l_tobytes(lua::State* L) {
    std::string_view string = lua::require_lstring(L, 1);
    if (lua::toboolean(L, 2)) {
        lua::createtable(L, string.length(), 0);
        for (size_t i = 0; i < string.length(); i++) {
            lua::pushinteger(L, string[i] & 0xFF);
            lua::rawseti(L, i+1);
        }
        return 1;
    } else {
        return lua::create_bytearray(L, string.data(), string.length());
    }
}

static int l_tostring(lua::State* L) {
    if (lua::istable(L, 1)) {
        lua::pushvalue(L, 1);
        size_t size = lua::objlen(L, 1);
        util::Buffer<char> buffer(size);
        for (size_t i = 0; i < size; i++) {
            lua::rawgeti(L, i + 1);
            buffer[i] = lua::tointeger(L, -1);
            lua::pop(L);
        }
        lua::pop(L);
        return lua::pushlstring(L, buffer.data(), size);
    } else {
        return lua::pushlstring(L, lua::bytearray_as_string(L, 1));
    }
}

static int l_length(lua::State* L) {
    auto string = lua::require_string(L, 1);
    return lua::pushinteger(L, util::length_utf8(string));
}

static int l_codepoint(lua::State* L) {
    std::string_view string = lua::require_string(L, 1);
    if (string.empty()) {
        return lua::pushinteger(L, 0);
    }
    uint size;
    return lua::pushinteger(L, util::decode_utf8(size, string.data()));
}

static int l_sub(lua::State* L) {
    auto string = util::str2u32str_utf8(lua::require_string(L, 1));
    int start = std::max(0, static_cast<int>(lua::tointeger(L, 2) - 1));
    int end = string.length();
    if (lua::gettop(L) >= 3) {
        end = std::max(0, static_cast<int>(lua::tointeger(L, 3) - 1));
    }
    return lua::pushstring(L, util::u32str2str_utf8(string.substr(start, end)));
}

static int l_upper(lua::State* L) {
    auto string = util::str2u32str_utf8(lua::require_string(L, 1));
    for (auto& c : string) {
        c = std::towupper(c);
    }
    return lua::pushstring(L, util::u32str2str_utf8(string));
}

static int l_lower(lua::State* L) {
    auto string = util::str2u32str_utf8(lua::require_string(L, 1));
    for (auto& c : string) {
        c = std::towlower(c);
    }
    return lua::pushstring(L, util::u32str2str_utf8(string));
}

static int l_encode(lua::State* L) {
    auto integer = lua::tointeger(L, 1);
    ubyte bytes[4];
    size_t count = util::encode_utf8(integer, bytes);
    return lua::pushlstring(L, bytes, count);
}

static int l_escape(lua::State* L) {
    auto string = lua::require_lstring(L, 1);
    return lua::pushstring(L, util::escape(string));
}

const luaL_Reg utf8lib[] = {
    {"tobytes", lua::wrap<l_tobytes>},
    {"tostring", lua::wrap<l_tostring>},
    {"length", lua::wrap<l_length>},
    {"codepoint", lua::wrap<l_codepoint>},
    {"sub", lua::wrap<l_sub>},
    {"upper", lua::wrap<l_upper>},
    {"lower", lua::wrap<l_lower>},
    {"encode", lua::wrap<l_encode>},
    {"escape", lua::wrap<l_escape>},
    {NULL, NULL}
};
