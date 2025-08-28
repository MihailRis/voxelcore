#include "api_lua.hpp"

#include <string>

#include "coders/byte_utils.hpp"
#include "util/data_io.hpp"

static size_t calc_size(const char* format) {
    size_t outSize = 0;
    for (size_t i = 0; format[i]; i++) {
        switch (format[i]) {
            case 'b':
            case 'B':
            case '?':
                outSize += 1;
                break;
            case 'h':
            case 'H':
                outSize += 2;
                break;
            case 'i':
            case 'I':
            case 'f':
                outSize += 4;
                break;
            case 'd':
            case 'l':
            case 'L':
                outSize += 8;
                break;
            default:
                break;
        }
    }
    return outSize;
}

static int pack(lua::State* L, const char* format, bool usetable) {
    size_t outSize = calc_size(format);
    bool bigEndian = false;

    ByteBuilder builder(outSize);
    int index = 2;
    for (int i = 0; format[i]; i++) {
        switch (format[i]) {
            case 'b':
                builder.put(lua::tointeger(L, index));
                break;
            case 'B':
                builder.put(lua::tointeger(L, index) & 0xFF);
                break;
            case '?':
                builder.put(lua::toboolean(L, index) ? 1 : 0);
                break;
            case 'h':
                builder.putInt16(lua::tointeger(L, index), bigEndian);
                break;
            case 'H':
                builder.putInt16(lua::tointeger(L, index) & 0xFFFFU, bigEndian);
                break;
            case 'i':
                builder.putInt32(lua::tointeger(L, index), bigEndian);
                break;
            case 'I':
                builder.putInt32(lua::tointeger(L, index) & 0xFFFFFFFFULL, bigEndian);
                break;
            case 'l':
            case 'L':
                builder.putInt64(lua::tointeger(L, index), bigEndian);
                break;
            case 'f':
                builder.putFloat32(lua::tonumber(L, index), bigEndian);
                break;
            case 'd':
                builder.putFloat64(lua::tonumber(L, index), bigEndian);
                break;
            case '!':
            case '>':
                bigEndian = true;
                continue;
            case '<':
                bigEndian = false;
                continue;
            case '@':
            case '=':
                bigEndian = dataio::is_big_endian();
                continue;
            default:
                continue;
        }
        index++;
    }
    if (usetable) {
        lua::createtable(L, outSize, 0);
        const ubyte* data = builder.data();
        for (size_t i = 0; i < outSize; i++) {
            lua::pushinteger(L, data[i]);
            lua::rawseti(L, i + 1);
        }
        return 1;
    } else {
        return lua::create_bytearray(L, builder.build());
    }
}

static int count_elements(const char* format) {
    int count = 0;
    for (size_t i = 0; format[i]; i++) {
        switch (format[i]) {
            case 'b':
            case 'B':
            case '?':
            case 'h':
            case 'H':
            case 'i':
            case 'I':
            case 'l':
            case 'L':
            case 'f':
            case 'd':
                count++;
                break;
            default:
                break;
        }
    }
    return count;
}

static int l_unpack(lua::State* L) {
    const char* format = lua::require_string(L, 1);
    int count = count_elements(format);
    auto bytes = lua::bytearray_as_string(L, 2);
    ByteReader reader(reinterpret_cast<const ubyte*>(bytes.data()), bytes.size());
    bool bigEndian = false;

    for (size_t i = 0; format[i]; i++) {
        switch (format[i]) {
            case 'b':
                lua::pushinteger(L, reader.get());
                break;
            case 'B':
                lua::pushinteger(L, reader.get() & 0xFF);
                break;
            case '?':
                lua::pushboolean(L, reader.get() != 0);
                break;
            case 'h':
                lua::pushinteger(L, reader.getInt16(bigEndian));
                break;
            case 'H':
                lua::pushinteger(L, reader.getInt16(bigEndian) & 0xFFFF);
                break;
            case 'i':
                lua::pushinteger(L, reader.getInt32(bigEndian));
                break;
            case 'I':
                lua::pushinteger(L, reader.getInt32(bigEndian) & 0xFFFFFFFF);
                break;
            case 'l':
                lua::pushinteger(L, reader.getInt64(bigEndian));
                break;
            case 'L':
                lua::pushinteger(L, reader.getInt64(bigEndian));
                break;
            case 'f':
                lua::pushnumber(L, reader.getFloat32(bigEndian));
                break;
            case 'd':
                lua::pushnumber(L, reader.getFloat64(bigEndian));
                break;
            case '!':
            case '>':
                bigEndian = true;
                continue;
            case '<':
                bigEndian = false;
                continue;
            case '@':
            case '=':
                bigEndian = dataio::is_big_endian();
                continue;
            default:
                continue;
        }
    }
    return count;
}

static int l_pack(lua::State* L) {
    const char* format = lua::require_string(L, 1);
    return pack(L, format, false);
}

static int l_tpack(lua::State* L) {
    const char* format = lua::require_string(L, 1);
    return pack(L, format, true);
}

static int l_get_size(lua::State* L) {
    return lua::pushinteger(
        L, static_cast<int>(calc_size(lua::require_string(L, 1)))
    );
}

const luaL_Reg byteutillib[] = {
    {"pack", lua::wrap<l_pack>},
    {"tpack", lua::wrap<l_tpack>},
    {"unpack", lua::wrap<l_unpack>},
    {"get_size", lua::wrap<l_get_size>},
    {NULL, NULL}
};
