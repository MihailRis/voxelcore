#include <iostream>
#include <iomanip>
#include <random>
#include <sstream>

#include "libs/api_lua.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "devtools/DebuggingServer.hpp"
#include "logic/scripting/scripting.hpp"

using namespace devtools;
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

constexpr inline int MAX_SHORT_STRING_LEN = 50;

static std::string get_short_value(lua::State* L, int idx, int type) {
    switch (type) {
        case LUA_TNIL:
            return "nil";
        case LUA_TBOOLEAN:
            return lua::toboolean(L, idx) ? "true" : "false";
        case LUA_TNUMBER: {
            std::stringstream ss;
            ss << lua::tonumber(L, idx);
            return ss.str();
        }
        case LUA_TSTRING: {
            const char* str = lua::tostring(L, idx);
            if (strlen(str) > MAX_SHORT_STRING_LEN) {
                return std::string(str, MAX_SHORT_STRING_LEN);
            } else {
                return str;
            }
        }
        case LUA_TTABLE:
            return "{...}";
        case LUA_TFUNCTION: {
            std::stringstream ss;
            ss << "function: 0x" << std::hex
               << reinterpret_cast<ptrdiff_t>(lua::topointer(L, idx));
            return ss.str();
        }
        case LUA_TUSERDATA: {
            std::stringstream ss;
            ss << "userdata: 0x" << std::hex
               << reinterpret_cast<ptrdiff_t>(lua::topointer(L, idx));
            return ss.str();
        }
        case LUA_TTHREAD: {
            std::stringstream ss;
            ss << "thread: 0x" << std::hex
               << reinterpret_cast<ptrdiff_t>(lua::topointer(L, idx));
            return ss.str();
        }
        default: {
            std::stringstream ss;
            ss << "cdata: 0x" << std::hex
               << reinterpret_cast<ptrdiff_t>(lua::topointer(L, idx));
            return ss.str();
        }
    }
}

static dv::value collect_locals(lua::State* L, lua_Debug& frame) {
    auto locals = dv::list();

    int localIndex = 1;
    const char* name;
    while ((name = lua_getlocal(L, &frame, localIndex++))) {
        if (name[0] == '(') {
            lua::pop(L);
            continue;
        }
        auto local = dv::object();
        local["name"] = name;
        local["index"] = localIndex - 1;

        int type = lua::type(L, -1);
        local["type"] = lua::type_name(L, type);
        local["short"] = get_short_value(L, -1, type);
        locals.add(std::move(local));
        lua::pop(L);
    }
    return locals;
}

static dv::value create_stack_trace(lua::State* L, int initFrame = 2) {
    auto entriesList = dv::list();

    lua_Debug frame;
    int level = initFrame;
    
    while (lua_getstack(L, level, &frame)) {
        auto entry = dv::object();
        if (lua_getinfo(L, "nSlf", &frame) == 0) {
            level++;
            entriesList.add(std::move(entry));
            continue;
        }
        if (frame.name) {
            entry["function"] = frame.name;
        }
        if (frame.source) {
            const char* src =
                (frame.source[0] == '@') ? frame.source + 1 : frame.source;
            entry["source"] = src;
            entry["line"] = frame.currentline;
        }
        entry["what"] = frame.what;
        entry["locals"] = collect_locals(L, frame);
        entriesList.add(std::move(entry));
        level++;
    }
    return entriesList;
}

static int l_debug_pause(lua::State* L) {
    if (auto server = engine->getDebuggingServer()) {
        std::string reason;
        std::string message;
        if (lua::isstring(L, 1)) {
            reason = lua::tolstring(L, 1);
        }
        if (lua::isstring(L, 2)) {
            message = lua::tolstring(L, 2);
        }
        server->pause(
            std::move(reason), std::move(message), create_stack_trace(L)
        );
    }
    return 0;
}

static int l_debug_sendvalue(lua::State* L) {
    auto server = engine->getDebuggingServer();
    if (!server) {
        return 0;
    }
    int frame = lua::tointeger(L, 2);
    int local = lua::tointeger(L, 3);

    ValuePath path;
    int pathSectors = lua::objlen(L, 4);
    for (int i = 0; i < pathSectors; i++) {
        lua::rawgeti(L, i + 1, 4);
        if (lua::isstring(L, -1)) {
            path.emplace_back(lua::tostring(L, -1));
        } else {
            path.emplace_back(static_cast<int>(lua::tointeger(L, -1)));
        }
        lua::pop(L);
    }

    dv::value value = nullptr;
    if (lua::istable(L, 1)) {
        auto table = dv::object();

        lua::pushnil(L);
        while (lua::next(L, 1)) {
            lua::pushvalue(L, -2);

            auto key = lua::tolstring(L, -1);
            int type = lua::type(L, -2);
            table[std::string(key)] = dv::object({
                {"type", std::string(lua::type_name(L, type))},
                {"short", get_short_value(L, -2, type)},
            });
            lua::pop(L, 2);
        }
        lua::pop(L);
        value = std::move(table);
    } else {
        value = lua::tovalue(L, 1);
    }

    server->sendValue(std::move(value), frame, local, std::move(path));
    return 0;
}

static int l_debug_pull_events(lua::State* L) {
    auto server = engine->getDebuggingServer();
    if (!server) {
        return 0;
    }
    auto events = server->pullEvents();
    if (events.empty()) {
        return 0;
    }
    lua::createtable(L, events.size(), 0);
    for (int i = 0; i < events.size(); i++) {
        const auto& event = events[i];
        lua::createtable(L, 3, 0);

        lua::pushinteger(L, static_cast<int>(event.type));
        lua::rawseti(L, 1);

        if (auto dto = std::get_if<BreakpointEventDto>(&event.data)) {
            lua::pushstring(L, dto->source);
            lua::rawseti(L, 2);

            lua::pushinteger(L, dto->line);
            lua::rawseti(L, 3);
        } else if (auto dto = std::get_if<GetValueEventDto>(&event.data)) {
            lua::pushinteger(L, dto->frame);
            lua::rawseti(L, 2);

            lua::pushinteger(L, dto->localIndex);
            lua::rawseti(L, 3);

            lua::createtable(L, dto->path.size(), 0);
            for (int i = 0; i < dto->path.size(); i++) {
                const auto& segment = dto->path[i];
                if (auto string = std::get_if<std::string>(&segment)) {
                    lua::pushstring(L, *string);
                } else {
                    lua::pushinteger(L, std::get<int>(segment));
                }
                lua::rawseti(L, i + 1);
            }
            lua::rawseti(L, 4);
        }

        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_debug_is_debugging(lua::State* L) {
    return lua::pushboolean(L, engine->getDebuggingServer() != nullptr);
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

        lua::pushcfunction(L, lua::wrap<l_debug_pause>);
        lua::setfield(L, "pause");

        lua::pushcfunction(L, lua::wrap<l_debug_pull_events>);
        lua::setfield(L, "__pull_events");

        lua::pushcfunction(L, lua::wrap<l_debug_sendvalue>);
        lua::setfield(L, "__sendvalue");

        lua::pushcfunction(L, lua::wrap<l_debug_is_debugging>);
        lua::setfield(L, "is_debugging");

        lua::pop(L);
    }
    if (lua::getglobal(L, "math")) {
        lua::pushcfunction(L, lua::wrap<l_math_normal_random>);
        lua::setfield(L, "normal_random");

        lua::pop(L);
    }
}
