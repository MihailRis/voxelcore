#include <iostream>
#include <iomanip>
#include <random>

#include "libs/api_lua.hpp"
#include "debug/Logger.hpp"
#include "logic/scripting/scripting.hpp"

using namespace scripting;

static debug::Logger logger("lua-debug");

static int l_debug_error(lua::State* L) {
    auto text = lua::require_string(L, 1);
    logger.error() << text;
    return 0;
}

static int l_debug_warning(lua::State* L) {
    auto text = lua::require_string(L, 1);
    logger.warning() << text;
    return 0;
}

static int l_debug_log(lua::State* L) {
    auto text = lua::require_string(L, 1);
    logger.info() << text;
    return 0;
}

const int MAX_DEPTH = 10;

int l_debug_print(lua::State* L) {
    auto addIndentation = [](int depth) {
        for (int i = 0; i < depth; ++i) *output_stream << "  ";
    };

    auto printHexData = [](const void* ptr, size_t size) {
        const auto* bytePtr = reinterpret_cast<const uint8_t*>(ptr);
        for (size_t i = 0; i < size; ++i) {
            *output_stream << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(bytePtr[i])
                      << ((i + 1) % 8 == 0 && i + 1 < size ? "\n" : " ");
        }
    };

    auto printEscapedString = [](const char* str) {
        while (*str) {
            switch (*str) {
                case '\\': *output_stream << "\\\\"; break;
                case '\"': *output_stream << "\\\""; break;
                case '\n': *output_stream << "\\n"; break;
                case '\t': *output_stream << "\\t"; break;
                case '\r': *output_stream << "\\r"; break;
                case '\b': *output_stream << "\\b"; break;
                case '\f': *output_stream << "\\f"; break;
                default:
                    if (iscntrl(static_cast<unsigned char>(*str))) {
                        // Print other control characters in \xHH format
                        *output_stream << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                                  << static_cast<int>(static_cast<unsigned char>(*str)) << std::dec;
                    } else {
                        *output_stream << *str;
                    }
                    break;
            }
            ++str;
        }
    };

    std::function<void(int, int, bool)> debugPrint = [&](int index,
                                                         int depth,
                                                         bool is_key) {
        if (depth > MAX_DEPTH) {
            *output_stream << "{...}";
            return;
        }
        switch (lua::type(L, index)) {
            case LUA_TSTRING:
                if (is_key){
                    *output_stream << lua::tostring(L, index);
                }else{
                    *output_stream << "\"";
                    printEscapedString(lua::tostring(L, index));
                    *output_stream << "\"";
                }
                break;
            case LUA_TBOOLEAN:
                *output_stream << (lua::toboolean(L, index) ? "true" : "false");
                break;
            case LUA_TNUMBER:
                *output_stream << lua::tonumber(L, index);
                break;
            case LUA_TTABLE: {
                bool is_list = lua::objlen(L, index) > 0, hadItems = false;
                int absTableIndex =
                    index > 0 ? index : lua::gettop(L) + index + 1;
                *output_stream << "{";
                lua::pushnil(L);
                while (lua::next(L, absTableIndex) != 0) {
                    if (hadItems)
                        *output_stream << "," << '\n';
                    else
                        *output_stream << '\n';

                    addIndentation(depth + 1);
                    if (!is_list) {
                        debugPrint(-2, depth, true);
                        *output_stream << " = ";
                    }
                    debugPrint(-1, depth + 1, false);
                    lua::pop(L, 1);
                    hadItems = true;
                }
                if (hadItems) *output_stream << '\n';
                addIndentation(depth);
                *output_stream << "}";
                break;
            }
            case LUA_TFUNCTION:
                *output_stream << "function(0x" << std::hex
                          << lua::topointer(L, index) << std::dec << ")";
                break;
            case LUA_TUSERDATA:
                *output_stream << "userdata:\n";
                printHexData(lua::topointer(L, index), lua::objlen(L, index));
                break;
            case LUA_TLIGHTUSERDATA:
                *output_stream << "lightuserdata:\n";
                printHexData(lua::topointer(L, index), sizeof(void*));
                break;
            case LUA_TNIL:
                *output_stream << "nil";
                break;
            default:
                *output_stream << lua::type_name(L, lua::type(L, index));
                break;
        }
    };

    int n = lua::gettop(L);
    *output_stream << "debug.print(" << '\n';
    for (int i = 1; i <= n; ++i) {
        addIndentation(1);
        debugPrint(i, 1, false);
        if (i < n) *output_stream << "," << '\n';
    }
    *output_stream << '\n' << ")" << std::endl;
    lua::pop(L, n);
    return 0;
}

namespace {
    std::normal_distribution<double> randomFloats(0.0f, 1.0f); 
    std::default_random_engine generator;
}

static int l_math_normal_random(lua::State* L) {
    return lua::pushnumber(L, randomFloats(generator));
}

void initialize_libs_extends(lua::State* L) {
    if (lua::getglobal(L, "debug")) {
        lua::pushcfunction(L, lua::wrap<l_debug_error>);
        lua::setfield(L, "error");

        lua::pushcfunction(L, lua::wrap<l_debug_warning>);
        lua::setfield(L, "warning");

        lua::pushcfunction(L, lua::wrap<l_debug_log>);
        lua::setfield(L, "log");

        lua::pushcfunction(L, lua::wrap<l_debug_print>);
        lua::setfield(L, "print");

        lua::pop(L);
    }
    if (lua::getglobal(L, "math")) {
        lua::pushcfunction(L, lua::wrap<l_math_normal_random>);
        lua::setfield(L, "normal_random");

        lua::pop(L);
    }
}
