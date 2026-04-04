#include "api_lua.hpp"
#include "coders/gzip.hpp"
#include "../lua_engine.hpp"

static int l_encode(lua::State* L) {
    char argc = lua::gettop(L);
    std::vector<unsigned char> compressedBytes;

    std::string algo = "gzip";
    if (argc >= 2) {
        if (!lua::isstring(L, 2)) {
            throw std::runtime_error("compression algorithm must be a string");
        }
        algo = lua::require_lstring(L, 2);
    }

    if (algo == "gzip") {
        auto str = lua::bytearray_as_string(L, 1);
        compressedBytes = gzip::compress(
            reinterpret_cast<const ubyte*>(str.data()),
            str.size()
        );
    } else {
        throw std::runtime_error("unsupported compression algorithm");
    }

    if (argc < 3 || !lua::toboolean(L, 3)) {
        lua::create_bytearray(L, std::move(compressedBytes));
    } else {
        size_t length = compressedBytes.size();
        lua::createtable(L, length, 0);
        int newTable = lua::gettop(L);
        for (size_t i = 0; i < length; i++) {
            lua::pushinteger(L, compressedBytes.data()[i]);
            lua::rawseti(L, i + 1, newTable);
        }
    }

    return 1;
}

static int l_decode(lua::State* L) {
    char argc = lua::gettop(L);
    std::vector<unsigned char> decompressedBytes;

    std::string algo = "gzip";
    if (argc >= 2) {
        if (!lua::isstring(L, 2)) {
            throw std::runtime_error("compression algorithm must be a string");
        }
        algo = lua::require_lstring(L, 2);
    }

    if (algo == "gzip") {
        auto str = lua::bytearray_as_string(L, 1);
        decompressedBytes = gzip::decompress(
            reinterpret_cast<const ubyte*>(str.data()),
            str.size()
        );
    } else {
        throw std::runtime_error("unsupported compression algorithm");
    }

    if (argc < 3 || !lua::toboolean(L, 3)) {
        lua::create_bytearray(L, std::move(decompressedBytes));
    } else {
        size_t length = decompressedBytes.size();
        lua::createtable(L, length, 0);
        int newTable = lua::gettop(L);
        for (size_t i = 0; i < length; i++) {
            lua::pushinteger(L, decompressedBytes.data()[i]);
            lua::rawseti(L, i + 1, newTable);
        }
    }

    return 1;
}

const luaL_Reg compressionlib[] = {
    {"encode", lua::wrap<l_encode>},
    {"decode", lua::wrap<l_decode>},
    {nullptr, nullptr}
};

