#pragma once

#include "delegates.hpp"
#include "logic/scripting/scripting.hpp"

#if (defined __linux__) || (defined __MINGW32__)
#include <luajit-2.1/luaconf.h>
#include <luajit-2.1/lua.hpp>
#else
#include <lua.hpp>
#endif
#include <glm/glm.hpp>
#include <stdexcept>
#include <string>

#ifndef LUAJIT_VERSION
#error LuaJIT required
#endif

namespace lua {
    class luaerror : public std::runtime_error {
    public:
        luaerror(const std::string& message);
    };

    using State = lua_State;
    using Number = lua_Number;
    using Integer = lua_Integer;

    /// @brief Automatically resets stack top element index to the initial state
    /// (when stackguard was created). Prevents Lua stack leak on exception
    /// occurred out of Lua execution time, when engine controls scripting.
    ///
    ///
    /// stackguard allows to not place lua::pop(...) into 'catch' blocks.
    class stackguard {
        int top;
        State* state;
    public:
        stackguard(State* state) : state(state) {
            top = lua_gettop(state);
        }

        ~stackguard() {
            lua_settop(state, top);
        }
    };

    void log_error(const std::string& text);

    class Userdata {
    public:
        virtual ~Userdata() {};
        virtual const std::string& getTypeName() const = 0;
    };
}
