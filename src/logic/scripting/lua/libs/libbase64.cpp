#include "api_lua.hpp"

#include "util/stringutil.hpp"

template<std::string(*encode_func)(const ubyte*, size_t)>
static int l_encode(lua::State* L) {
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
        return lua::pushstring(L, encode_func(
            reinterpret_cast<const ubyte*>(buffer.data()), buffer.size()
        ));
    } else {
        auto string = lua::bytearray_as_string(L, 1);
        auto out = encode_func(
            reinterpret_cast<const ubyte*>(string.data()),
            string.size()
        );
        lua::pop(L);
        return lua::pushstring(L, std::move(out));
    }
    throw std::runtime_error("array or ByteArray expected");
}

template<util::Buffer<ubyte>(*decode_func)(std::string_view)>
static int l_decode(lua::State* L) {
    auto buffer = decode_func(lua::require_lstring(L, 1));
    if (lua::toboolean(L, 2)) {
        lua::createtable(L, buffer.size(), 0);
        for (size_t i = 0; i < buffer.size(); i++) {
            lua::pushinteger(L, buffer[i] & 0xFF);
            lua::rawseti(L, i+1);
        }
        return 1;
    } else {
        return lua::create_bytearray(L, buffer.data(), buffer.size());
    }
}

const luaL_Reg base64lib[] = {
    {"encode", lua::wrap<l_encode<util::base64_encode>>},
    {"decode", lua::wrap<l_decode<util::base64_decode>>},
    {"encode_urlsafe", lua::wrap<l_encode<util::base64_urlsafe_encode>>},
    {"decode_urlsafe", lua::wrap<l_decode<util::base64_urlsafe_decode>>},
    {NULL, NULL}
};
