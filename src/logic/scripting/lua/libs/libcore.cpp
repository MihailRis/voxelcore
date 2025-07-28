#include <memory>
#include <vector>
#include <sstream>

#include "api_lua.hpp"
#include "constants.hpp"
#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "content/ContentControl.hpp"
#include "engine/Engine.hpp"
#include "io/engine_paths.hpp"
#include "io/io.hpp"
#include "io/settings_io.hpp"
#include "frontend/menu.hpp"
#include "frontend/screens/MenuScreen.hpp"
#include "graphics/core/Texture.hpp"
#include "logic/EngineController.hpp"
#include "logic/LevelController.hpp"
#include "util/listutil.hpp"
#include "util/platform.hpp"
#include "world/Level.hpp"
#include "world/generator/WorldGenerator.hpp"

using namespace scripting;

static int l_get_version(lua::State* L) {
    return lua::pushvec_stack(
        L, glm::vec2(ENGINE_VERSION_MAJOR, ENGINE_VERSION_MINOR)
    );
}

static int l_load_content(lua::State* L) {
    content_control->loadContent();
    return 0;
}

static int l_reset_content(lua::State* L) {
    if (level != nullptr) {
        throw std::runtime_error("world must be closed before");
    }
    content_control->resetContent();
    return 0;
}

static int l_is_content_loaded(lua::State* L) {
    return lua::pushboolean(L, content != nullptr);
}

/// @brief Creating new world
/// @param name Name world
/// @param seed Seed world
/// @param generator Type of generation
static int l_new_world(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto seed = lua::require_string(L, 2);
    auto generator = lua::require_string(L, 3);
    int64_t localPlayer = 0;
    if (lua::gettop(L) >= 4) {
        localPlayer = lua::tointeger(L, 4);
    }
    if (level != nullptr) {
        throw std::runtime_error("world must be closed before");
    }
    auto controller = engine->getController();
    controller->setLocalPlayer(localPlayer);
    controller->createWorld(name, seed, generator);
    return 0;
}

/// @brief Open world
/// @param name Name world
static int l_open_world(lua::State* L) {
    auto name = lua::require_string(L, 1);
    if (level != nullptr) {
        throw std::runtime_error("world must be closed before");
    }
    auto controller = engine->getController();
    controller->setLocalPlayer(0);
    controller->openWorld(name, false);
    return 0;
}

/// @brief Reopen world
static int l_reopen_world(lua::State*) {
    auto controller = engine->getController();
    if (level == nullptr) {
        throw std::runtime_error("no world open");
    }
    controller->reopenWorld(level->getWorld());
    return 0;
}

/// @brief Save world
static int l_save_world(lua::State* L) {
    if (controller == nullptr) {
        throw std::runtime_error("no world open");
    }
    controller->saveWorld();
    return 0;
}

/// @brief Close world
/// @param flag Save world (bool)
static int l_close_world(lua::State* L) {
    if (controller == nullptr) {
        throw std::runtime_error("no world open");
    }
    bool save_world = lua::toboolean(L, 1);
    if (save_world) {
        controller->saveWorld();
    }
    engine->onWorldClosed();
    return 0;
}

/// @brief Delete world
/// @param name Name world
static int l_delete_world(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto controller = engine->getController();
    controller->deleteWorld(name);
    return 0;
}

/// @brief Reconfigure packs
/// @param addPacks An array of packs to add
/// @param remPacks An array of packs to remove
static int l_reconfig_packs(lua::State* L) {
    if (!lua::istable(L, 1)) {
        throw std::runtime_error("strings array expected as the first argument");
    }
    if (!lua::istable(L, 2)) {
        throw std::runtime_error("strings array expected as the second argument");
    }
    std::vector<std::string> addPacks;
    int addLen = lua::objlen(L, 1);
    for (int i = 0; i < addLen; i++) {
        lua::rawgeti(L, i + 1, 1);
        addPacks.emplace_back(lua::require_lstring(L, -1));
        lua::pop(L);
    }
    std::vector<std::string> remPacks;
    int remLen = lua::objlen(L, 2);
    for (int i = 0; i < remLen; i++) {
        lua::rawgeti(L, i + 1, 2);
        remPacks.emplace_back(lua::require_lstring(L, -1));
        lua::pop(L);
    }
    auto engineController = engine->getController();
    try {
        engineController->reconfigPacks(controller, addPacks, remPacks);
    } catch (const contentpack_error& err) {
        throw std::runtime_error(
            std::string(err.what()) + " [" + err.getPackId() + " ]"
        );
    }
    return 0;
}

/// @brief Get a setting value
/// @param name The name of the setting
/// @return The value of the setting
static int l_get_setting(lua::State* L) {
    auto name = lua::require_string(L, 1);
    const auto value = engine->getSettingsHandler().getValue(name);
    return lua::pushvalue(L, value);
}

/// @brief Set a setting value
/// @param name The name of the setting
/// @param value The new value for the setting
static int l_set_setting(lua::State* L) {
    auto name = lua::require_string(L, 1);
    const auto value = lua::tovalue(L, 2);
    engine->getSettingsHandler().setValue(name, value);
    return 0;
}

/// @brief Convert a setting value to a string
/// @param name The name of the setting
/// @return The string representation of the setting value
static int l_str_setting(lua::State* L) {
    auto name = lua::require_string(L, 1);
    const auto string = engine->getSettingsHandler().toString(name);
    return lua::pushstring(L, string);
}

/// @brief Get information about a setting
/// @param name The name of the setting
/// @return A table with information about the setting
static int l_get_setting_info(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto setting = engine->getSettingsHandler().getSetting(name);
    lua::createtable(L, 0, 1);
    if (auto number = dynamic_cast<NumberSetting*>(setting)) {
        lua::pushnumber(L, number->getMin());
        lua::setfield(L, "min");
        lua::pushnumber(L, number->getMax());
        lua::setfield(L, "max");
        lua::pushnumber(L, number->getDefault());
        lua::setfield(L, "def");
        return 1;
    }
    if (auto integer = dynamic_cast<IntegerSetting*>(setting)) {
        lua::pushinteger(L, integer->getMin());
        lua::setfield(L, "min");
        lua::pushinteger(L, integer->getMax());
        lua::setfield(L, "max");
        lua::pushinteger(L, integer->getDefault());
        lua::setfield(L, "def");
        return 1;
    }
    if (auto boolean = dynamic_cast<FlagSetting*>(setting)) {
        lua::pushboolean(L, boolean->getDefault());
        lua::setfield(L, "def");
        return 1;
    }
    if (auto string = dynamic_cast<StringSetting*>(setting)) {
        lua::pushstring(L, string->getDefault());
        lua::setfield(L, "def");
        return 1;
    }
    lua::pop(L);
    throw std::runtime_error("unsupported setting type");
}

static int l_open_folder(lua::State* L) {
    platform::open_folder(io::resolve(lua::require_string(L, 1)));
    return 0;
}

/// @brief Quit the game
static int l_quit(lua::State*) {
    engine->quit();
    return 0;
}

static int l_blank(lua::State*) {
    return 0;
}

static int l_capture_output(lua::State* L) {
    int argc = lua::gettop(L) - 1;
    if (!lua::isfunction(L, 1)) {
        throw std::runtime_error("function expected as argument 1");
    }
    for (int i = 0; i < argc; i++) {
        lua::pushvalue(L, i + 2);
    }
    lua::pushvalue(L, 1);

    auto prev_output = output_stream;
    auto prev_error = error_stream;

    std::stringstream captured_output;

    output_stream = &captured_output;
    error_stream = &captured_output;
    
    lua::call_nothrow(L, argc, 0);

    output_stream = prev_output;
    error_stream = prev_error;
    
    lua::pushstring(L, captured_output.str());
    return 1;
}

const luaL_Reg corelib[] = {
    {"blank", lua::wrap<l_blank>},
    {"get_version", lua::wrap<l_get_version>},
    {"load_content", lua::wrap<l_load_content>},
    {"reset_content", lua::wrap<l_reset_content>},
    {"is_content_loaded", lua::wrap<l_is_content_loaded>},
    {"new_world", lua::wrap<l_new_world>},
    {"open_world", lua::wrap<l_open_world>},
    {"reopen_world", lua::wrap<l_reopen_world>},
    {"save_world", lua::wrap<l_save_world>},
    {"close_world", lua::wrap<l_close_world>},
    {"delete_world", lua::wrap<l_delete_world>},
    {"reconfig_packs", lua::wrap<l_reconfig_packs>},
    {"get_setting", lua::wrap<l_get_setting>},
    {"set_setting", lua::wrap<l_set_setting>},
    {"str_setting", lua::wrap<l_str_setting>},
    {"get_setting_info", lua::wrap<l_get_setting_info>},
    {"open_folder", lua::wrap<l_open_folder>},
    {"quit", lua::wrap<l_quit>},
    {"capture_output", lua::wrap<l_capture_output>},
    {NULL, NULL}
};
