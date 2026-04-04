#pragma once

#include "lua_commons.hpp"

namespace lua {
    template <lua_CFunction func>
    int wrap(lua_State* L) {
        int result = 0;
        try {
            result = func(L);
        }
        // transform exception with description into lua_error
        catch (std::exception& e) {
            luaL_error(L, e.what());
        }
        // Rethrow any other exception (lua error for example)
        catch (...) {
            throw;
        }
        return result;
    }

    inline void pop(lua::State* L, int n = 1) {
#ifndef NDEBUG
        if (n < 0) {
            abort();
        }
        int top = lua_gettop(L);
        if (top < n) {
            abort();
        }
#endif
        lua_pop(L, n);
    }
    inline void insert(lua::State* L, int idx) {
        lua_insert(L, idx);
    }
    inline void remove(lua::State* L, int idx) {
        lua_remove(L, idx);
    }
    inline int gettop(lua::State* L) {
        return lua_gettop(L);
    }
    inline size_t objlen(lua::State* L, int idx) {
        return lua_objlen(L, idx);
    }
    inline int next(lua::State* L, int idx) {
        return lua_next(L, idx);
    }
    inline int type(lua::State* L, int idx) {
        return lua_type(L, idx);
    }
    inline const char* type_name(lua::State* L, int tp) {
        return lua_typename(L, tp);
    }
    inline int rawget(lua::State* L, int idx = -2) {
        lua_rawget(L, idx);
        return 1;
    }
    inline int rawgeti(lua::State* L, int n, int idx = -1) {
        lua_rawgeti(L, idx, n);
        return 1;
    }
    inline void rawseti(lua::State* L, int n, int idx = -2) {
        lua_rawseti(L, idx, n);
    }
    inline void rawset(lua::State* L, int idx = -3) {
        lua_rawset(L, idx);
    }

    inline int createtable(lua::State* L, int narr, int nrec) {
        lua_createtable(L, narr, nrec);
        return 1;
    }

    inline bool isnil(lua::State* L, int idx) {
        return lua_isnil(L, idx);
    }

    // function wrappers with number of pushed values as return value

    inline int pushnil(lua::State* L) {
        lua_pushnil(L);
        return 1;
    }

    inline int pushinteger(lua::State* L, lua::Integer x) {
        lua_pushinteger(L, x);
        return 1;
    }

    inline int pushnumber(lua::State* L, lua::Number x) {
        lua_pushnumber(L, x);
        return 1;
    }

}
