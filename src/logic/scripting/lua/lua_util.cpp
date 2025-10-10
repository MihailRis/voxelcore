#include "lua_util.hpp"
#include "lua_engine.hpp"

#include <iomanip>
#include <iostream>

#include "util/stringutil.hpp"

using namespace lua;

static int nextEnvironment = 1;

std::unordered_map<std::type_index, std::string> lua::usertypeNames;

int lua::userdata_destructor(lua::State* L) {
    if (auto obj = touserdata<Userdata>(L, 1)) {
        obj->~Userdata();
    }
    return 0;
}

std::string lua::env_name(int env) {
    return "_ENV" + util::mangleid(env);
}

int lua::pushvalue(State* L, const dv::value& value) {
    using dv::value_type;

    switch (value.getType()) {
        case value_type::none:
            pushnil(L);
            break;
        case value_type::boolean:
            pushboolean(L, value.asBoolean());
            break;
        case value_type::number:
            pushnumber(L, value.asNumber());
            break;
        case value_type::integer:
            pushinteger(L, value.asInteger());
            break;
        case value_type::string:
            pushstring(L, value.asString());
            break;
        case value_type::list: {
            createtable(L, value.size(), 0);
            size_t index = 1;
            for (const auto& elem : value) {
                pushvalue(L, elem);
                rawseti(L, index);
                index++;
            }
            break;
        }
        case value_type::object:
            createtable(L, 0, value.size());
            for (const auto& [key, elem] : value.asObject()) {
                pushvalue(L, elem);
                setfield(L, key);
            }
            break;
        case value_type::bytes: {
            const auto& bytes = value.asBytes();
            create_bytearray(L, bytes.data(), bytes.size());
            break;
        }
    }
    return 1;
}

std::wstring lua::require_wstring(State* L, int idx) {
    return util::str2wstr_utf8(require_string(L, idx));
}

int lua::pushwstring(State* L, const std::wstring& str) {
    return pushstring(L, util::wstr2str_utf8(str));
}

dv::value lua::tovalue(State* L, int idx) {
    using dv::value_type;
    auto type = lua::type(L, idx);
    switch (type) {
        case LUA_TNIL:
        case LUA_TNONE:
            return nullptr;
        case LUA_TBOOLEAN:
            return toboolean(L, idx) == 1;
        case LUA_TNUMBER: {
            auto number = tonumber(L, idx);
            auto integer = tointeger(L, idx);
            if (number == static_cast<Number>(integer)) {
                return integer;
            } else {
                return number;
            }
        }
        case LUA_TFUNCTION:
            return "<function " +
                   std::to_string(
                       reinterpret_cast<ptrdiff_t>(lua_topointer(L, idx))
                   ) +
                   ">";
        case LUA_TSTRING:
            return std::string(tostring(L, idx));
        case LUA_TTABLE: {
            int len = objlen(L, idx);
            if (len) {
                // array
                auto list = dv::list();
                for (int i = 1; i <= len; i++) {
                    rawgeti(L, i, idx);
                    list.add(tovalue(L, -1));
                    pop(L);
                }
                return list;
            } else {
                // table
                auto map = dv::object();
                pushvalue(L, idx);
                pushnil(L);
                while (next(L, -2)) {
                    pushvalue(L, -2);
                    auto key = tostring(L, -1);
                    map[key] = tovalue(L, -2);
                    pop(L, 2);
                }
                pop(L);
                return map;
            }
        }
        default: {
            auto data = bytearray_as_string(L, idx);
            auto bytes = std::make_shared<dv::objects::Bytes>(
                reinterpret_cast<const ubyte*>(data.data()), data.size()
            );
            return bytes;
        }
    }
}

static int l_error_handler(lua_State* L) {
    if (!isstring(L, 1)) {  // 'message' not a string?
        return 1;           // keep it intact
    }
    if (getglobal(L, "__vc__error")) {
        lua_pushvalue(L, 1);    // pass error message
        lua_pushinteger(L, 2);  // skip this function and traceback
        lua_call(L, 2, 1);      // call debug.traceback
    } if (get_from(L, "debug", "traceback")) {
        lua_pushvalue(L, 1);
        lua_pushinteger(L, 2);
        lua_call(L, 2, 1);
    }
    return 1;
}

int lua::call(State* L, int argc, int nresults) {
    int handler_pos = gettop(L) - argc;
    pushcfunction(L, l_error_handler);
    insert(L, handler_pos);
    int top = gettop(L);
    if (lua_pcall(L, argc, nresults, handler_pos)) {
        std::string log = tostring(L, -1);
        pop(L);
        remove(L, handler_pos);
        throw luaerror(log);
    }
    int added = gettop(L) - (top - argc - 1);
    remove(L, handler_pos);
    return added;
}

int lua::call_nothrow(State* L, int argc, int nresults) {
    int handler_pos = gettop(L) - argc;
    pushcfunction(L, l_error_handler);
    insert(L, handler_pos);
    int top = gettop(L);
    if (lua_pcall(L, argc, -1, handler_pos)) {
        auto errorstr = tostring(L, -1);
        if (errorstr) {
            log_error(errorstr);
            pop(L);
        } else {
            log_error("");
        }
        remove(L, handler_pos);
        return 0;
    }
    int added = gettop(L) - (top - argc - 1);
    remove(L, handler_pos);
    return added;
}

void lua::dump_stack(State* L) {
    int top = gettop(L);
    for (int i = 1; i <= top; i++) {
        std::cout << std::setw(3) << i << std::setw(20) << luaL_typename(L, i)
                  << std::setw(30);
        switch (lua::type(L, i)) {
            case LUA_TNUMBER:
                std::cout << tonumber(L, i);
                break;
            case LUA_TSTRING:
                std::cout << tostring(L, i);
                break;
            case LUA_TBOOLEAN:
                std::cout << (toboolean(L, i) ? "true" : "false");
                break;
            case LUA_TNIL:
                std::cout << "nil";
                break;
            default:
                std::cout << topointer(L, i);
                break;
        }
        std::cout << std::endl;
    }
}

static std::shared_ptr<std::string> create_lambda_handler(State* L) {
    auto ptr = reinterpret_cast<ptrdiff_t>(topointer(L, -1));
    auto name = util::mangleid(ptr);
    requireglobal(L, LAMBDAS_TABLE);
    pushvalue(L, -2);
    setfield(L, name);
    pop(L, 2);

    return std::shared_ptr<std::string>(
        new std::string(name),
        [=](std::string* name) {
            auto L = lua::get_main_state();
            requireglobal(L, LAMBDAS_TABLE);
            pushnil(L);
            setfield(L, *name);
            pop(L);
            delete name;
        }
    );
}

runnable lua::create_runnable(State* L) {
    auto funcptr = create_lambda_handler(L);
    return [=]() {
        auto L = lua::get_main_state();
        if (!get_from(L, LAMBDAS_TABLE, *funcptr, false))
            return;
        call_nothrow(L, 0, 0);
        pop(L);
    };
}

KeyCallback lua::create_simple_handler(State* L) {
    auto funcptr = create_lambda_handler(L);
    return [=]() -> bool {
        if (!get_from(L, LAMBDAS_TABLE, *funcptr, false))
            return false;
        int top = gettop(L) - 1;
        if (call_nothrow(L, 0)) {
            int nres = gettop(L) - top;
            if (nres) {
                bool result = toboolean(L, -1);
                pop(L, 1 + nres);
                return result;
            }
        }
        pop(L);
        return false;
    };
}

scripting::common_func lua::create_lambda(State* L) {
    auto funcptr = create_lambda_handler(L);
    return [=](const std::vector<dv::value>& args) -> dv::value {
        if (!get_from(L, LAMBDAS_TABLE, *funcptr, false))
            return nullptr;
        int top = gettop(L) - 1;
        for (const auto& arg : args) {
            pushvalue(L, arg);
        }
        if (call(L, args.size(), 1)) {
            int nres = gettop(L) - top;
            assert(nres >= 0);
            if (nres) {
                auto result = tovalue(L, -1);
                pop(L, 1 + nres);
                return result;
            }
        }
        pop(L);
        return nullptr;
    };
}

scripting::common_func lua::create_lambda_nothrow(State* L) {
    auto funcptr = create_lambda_handler(L);
    return [=](const std::vector<dv::value>& args) -> dv::value {
        if (!get_from(L, LAMBDAS_TABLE, *funcptr, false))
            return nullptr;
        int top = gettop(L) - 1;
        for (const auto& arg : args) {
            pushvalue(L, arg);
        }
        if (call_nothrow(L, args.size(), 1)) {
            int nres = gettop(L) - top;
            if (nres) {
                auto result = tovalue(L, -1);
                pop(L, 1 + nres);
                return result;
            }
        }
        pop(L);
        return nullptr;
    };
}

int lua::create_environment(State* L, int parent) {
    int id = nextEnvironment++;

    // local env = {}
    createtable(L, 0, 1);

    // setmetatable(env, {__index=_G})
    createtable(L, 0, 1);
    if (parent == 0) {
        pushglobals(L);
    } else {
        if (pushenv(L, parent) == 0) {
            pushglobals(L);
        }
    }
    setfield(L, "__index");
    setmetatable(L);

    // envname = env
    setglobal(L, env_name(id));
    return id;
}

void lua::remove_environment(State* L, int id) {
    if (id == 0) {
        return;
    }
    pushnil(L);
    setglobal(L, env_name(id));
}
