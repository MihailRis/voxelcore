#pragma once

#include <exception>
#include <string>

#include "../lua_util.hpp"

/// Definitions can be found in local .cpp files
/// having same names as declarations

/// l_ prefix means that function is lua_CFunction:
///    int l_function_name(lua_State* L);
/// use following syntax:
///    int l_function_name(lua::State* L);

// Libraries
extern const luaL_Reg applib[];
extern const luaL_Reg assetslib[];
extern const luaL_Reg audiolib[];
extern const luaL_Reg base64lib[];
extern const luaL_Reg bjsonlib[];
extern const luaL_Reg blocklib[];
extern const luaL_Reg blockwrapslib[]; // gfx.blockwraps
extern const luaL_Reg byteutillib[];
extern const luaL_Reg cameralib[];
extern const luaL_Reg consolelib[];
extern const luaL_Reg corelib[];
extern const luaL_Reg entitylib[];
extern const luaL_Reg filelib[];
extern const luaL_Reg generationlib[];
extern const luaL_Reg guilib[];
extern const luaL_Reg hudlib[];
extern const luaL_Reg inputlib[];
extern const luaL_Reg inventorylib[];
extern const luaL_Reg itemlib[];
extern const luaL_Reg jsonlib[];
extern const luaL_Reg mat4lib[];
extern const luaL_Reg networklib[];
extern const luaL_Reg packlib[];
extern const luaL_Reg particleslib[]; // gfx.particles
extern const luaL_Reg pathfindinglib[];
extern const luaL_Reg playerlib[];
extern const luaL_Reg posteffectslib[]; // gfx.posteffects
extern const luaL_Reg quatlib[];
extern const luaL_Reg randomlib[];
extern const luaL_Reg text3dlib[]; // gfx.text3d
extern const luaL_Reg timelib[];
extern const luaL_Reg tomllib[];
extern const luaL_Reg utf8lib[];
extern const luaL_Reg vec2lib[];  // vecn.cpp
extern const luaL_Reg vec3lib[];  // vecn.cpp
extern const luaL_Reg vec4lib[];  // vecn.cpp
extern const luaL_Reg weatherlib[]; // gfx.weather
extern const luaL_Reg worldlib[];
extern const luaL_Reg yamllib[];

// Components
extern const luaL_Reg skeletonlib[];
extern const luaL_Reg rigidbodylib[];
extern const luaL_Reg transformlib[];

// Lua Overrides
extern int l_print(lua::State* L);
extern int l_crc32(lua::State* L);

namespace lua {
    inline uint check_argc(lua::State* L, int a) {
        int argc = lua::gettop(L);
        if (argc == a) {
            return static_cast<uint>(argc);
        } else {
            throw std::runtime_error(
                "invalid number of arguments (" + std::to_string(a) +
                " expected)"
            );
        }
    }

    [[nodiscard]] inline uint check_argc(lua::State* L, int a, int b) {
        int argc = lua::gettop(L);
        if (argc == a || argc == b) {
            return static_cast<uint>(argc);
        } else {
            throw std::runtime_error(
                "invalid number of arguments (" + std::to_string(a) + " or " +
                std::to_string(b) + " expected)"
            );
        }
    }
}

void initialize_libs_extends(lua::State* L);
