#include "lua_engine.hpp"

#include <iomanip>
#include <iostream>

#include "io/io.hpp"
#include "engine/EnginePaths.hpp"
#include "debug/Logger.hpp"
#include "util/stringutil.hpp"
#include "libs/api_lua.hpp"
#include "usertypes/lua_type_heightmap.hpp"
#include "usertypes/lua_type_voxelfragment.hpp"
#include "usertypes/lua_type_canvas.hpp"
#include "usertypes/lua_type_random.hpp"
#include "usertypes/lua_type_pcmstream.hpp"
#include "engine/Engine.hpp"

namespace {
    debug::Logger logger("lua-state");
    lua::State* main_thread = nullptr;
    bool headless_mode = false;
    const std::unordered_map<std::string, std::string>* project_args;
}

using namespace lua;

luaerror::luaerror(const std::string& message) : std::runtime_error(message) {
}

static void remove_lib_funcs(
    State* L, const char* libname, const char* funcs[]
) {
    if (getglobal(L, libname)) {
        for (uint i = 0; funcs[i]; i++) {
            pushnil(L);
            setfield(L, funcs[i], -2);
        }
        pop(L);
    }
}

[[nodiscard]] scriptenv lua::create_environment(State* L) {
    int id = lua::create_environment(L, 0);
    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

static void create_libs(State* L, StateType stateType) {
    openlib(L, "base64", base64lib);
    openlib(L, "bjson", bjsonlib);
    openlib(L, "block", blocklib);
    openlib(L, "byteutil", byteutillib);
    openlib(L, "file", filelib);
    openlib(L, "generation", generationlib);
    openlib(L, "item", itemlib);
    openlib(L, "json", jsonlib);
    openlib(L, "mat4", mat4lib);
    openlib(L, "pack", packlib);
    openlib(L, "quat", quatlib);
    openlib(L, "random", randomlib);
    openlib(L, "compression", compressionlib);
    openlib(L, "toml", tomllib);
    openlib(L, "utf8", utf8lib);
    openlib(L, "vec2", vec2lib);
    openlib(L, "vec3", vec3lib);
    openlib(L, "vec4", vec4lib);
    openlib(L, "xml", xmllib);
    openlib(L, "yaml", yamllib);

    openlib(L, "__vc_app", applib);
    lua::getglobal(L, "__vc_app");
    lua::setregistry(L, "app");

    if (stateType == StateType::SCRIPT) {
        lua::getregistry(L, "app");
        lua::setglobal(L, "app");
    }
    if (stateType == StateType::BASE || stateType == StateType::SCRIPT) {
        openlib(L, "assets", assetslib);
        openlib(L, "audio", audiolib);
        openlib(L, "console", consolelib);
        openlib(L, "core", corelib);
        openlib(L, "gui", guilib);
        openlib(L, "input", inputlib);
        openlib(L, "inventory", inventorylib);
        openlib(L, "network", networklib);
        openlib(L, "pathfinding", pathfindinglib);
        openlib(L, "player", playerlib);
        openlib(L, "time", timelib);
        openlib(L, "world", worldlib);

        openlib(L, "entities", entitylib);
        openlib(L, "cameras", cameralib);

        // components
        openlib(L, "__skeleton", skeletonlib);
        openlib(L, "__rigidbody", rigidbodylib);
        openlib(L, "__transform", transformlib);
    }

    addfunc(L, "print", lua::wrap<l_print>);
    addfunc(L, "crc32", lua::wrap<l_crc32>);
}

static int l_panic_handler(lua::State* L) {
    logger.error() << "PANIC: unprotected error in call to Lua API: " << lua::tostring(L, -1);
    logger.flush();
    abort();
}

void lua::init_state(State* L, StateType stateType) {
    lua_atpanic(L, l_panic_handler);

    // Allowed standard libraries
    luaL_openlibs(L);

    if (getglobal(L, "require")) {
        pushstring(L, "ffi");
        if (call_nothrow(L, 1, 1)) {
            setglobal(L, "ffi");
        }
    }
    pushnil(L);
    setglobal(L, "io");

    createtable(L, 0, 0);
    pushvalue(L, -1);
    setglobal(L, "__vc__pack_envs");
    setregistry(L, lua::PACK_ENVS_TABLE);

    const char* removed_os[] {
        "execute", "exit", "remove", "rename", "setlocale", "tmpname", nullptr};
    remove_lib_funcs(L, "os", removed_os);
    create_libs(L, stateType);

    createtable(L, 0, 0);
    setregistry(L, LAMBDAS_TABLE);

    createtable(L, 0, 0);
    setregistry(L, CHUNKS_TABLE);

    createtable(L, 0, 0);
    pushglobals(L);
    setfield(L, env_name(0));
    setregistry(L, ENVS_TABLE);

    initialize_libs_extends(L);

    newusertype<LuaHeightmap>(L);
    newusertype<LuaVoxelFragment>(L);
    newusertype<LuaCanvas>(L);

    pushboolean(L, headless_mode);
    setglobal(L, "__VC_HEADLESS");

    createtable(L, 0, project_args->size());
    for (const auto& [key, value] : *project_args) {
        pushstring(L, value);
        setfield(L, key);
    }
    setglobal(L, "__VC_PROJECT_ARGS");

    auto file = "res:scripts/stdmin.lua";
    auto src = io::read_string(file);
    lua::pop(L, lua::execute(L, 0, src, "core:scripts/stdmin.lua"));

    newusertype<LuaRandom>(L);
    if (getglobal(L, "random")) {
        if (getglobal(L, "__vc_Random")) {
            setfield(L, "Random");
        }
        pop(L);
    }
    newusertype<LuaPCMStream>(L);
    if (getglobal(L, "audio")) {
        if (getglobal(L, "__vc_PCMStream")) {
            setfield(L, "PCMStream");
        }
        pop(L);
    }

    if (stateType == StateType::GENERATOR) {
        pushnil(L);
        setglobal(L, "ffi");
    }
}

void lua::initialize(const EnginePaths& paths, const CoreParameters& params) {
    logger.info() << LUA_VERSION;
    logger.info() << LUAJIT_VERSION;

    headless_mode = params.headless;
    project_args = &params.projectArgs;
    main_thread = create_state(
        paths, params.headless ? StateType::SCRIPT : StateType::BASE
    );
    lua::pushstring(main_thread, params.scriptFile.stem().u8string());
    lua::setglobal(main_thread, "__VC_SCRIPT_NAME");
}

void lua::finalize() {
    lua::close(main_thread);
}

bool lua::emit_event(
    State* L, const std::string& name, std::function<int(State*)> args
) {
    getglobal(L, "events");
    getfield(L, "emit");
    pushstring(L, name);
    if (call_nothrow(L, args(L) + 1)) {
        bool result = toboolean(L, -1);
        pop(L, 2);
        return result;
    }
    pop(L, 1);
    return false;
}

State* lua::get_main_state() {
    return main_thread;
}

State* lua::create_state(const EnginePaths& paths, StateType stateType) {
    auto L = luaL_newstate();
    if (L == nullptr) {
        throw luaerror("could not initialize Lua state");
    }
    init_state(L, stateType);
    return L;
}
