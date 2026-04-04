#include "api_lua.hpp"

#include "content/ContentControl.hpp"
#include "devtools/Project.hpp"
#include "engine/Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "frontend/locale.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/ui/gui_util.hpp"
#include "graphics/ui/GUI.hpp"
#include "io/devices/MemoryDevice.hpp"
#include "io/io.hpp"
#include "io/settings_io.hpp"
#include "logic/EngineController.hpp"
#include "logic/LevelController.hpp"
#include "logic/scripting/scripting.hpp"
#include "network/Network.hpp"
#include "util/platform.hpp"
#include "util/stringutil.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"

using namespace scripting;

/// @brief Check if content is loaded
static int l_is_content_loaded(lua::State* L) {
    return lua::pushboolean(L, content != nullptr);
}

/// @brief Load content
static int l_load_content(lua::State* L) {
    content_control->loadContent();
    return 0;
}

/// @brief Reset content, excluding specified packs modules
static int l_reset_content(lua::State* L) {
    if (level != nullptr) {
        throw std::runtime_error("world must be closed before");
    }
    std::vector<std::string> nonResetPacks;
    if (lua::istable(L, 1)) {
        int len = lua::objlen(L, 1);
        for (int i = 0; i < len; i++) {
            lua::rawgeti(L, i + 1, 1);
            nonResetPacks.emplace_back(lua::require_lstring(L, -1));
            lua::pop(L);
        }
    }
    content_control->resetContent(std::move(nonResetPacks));
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

/// @brief Get content sources list
static int l_get_content_sources(lua::State* L) {
    const auto& sources = engine->getContentControl().getContentSources();
    lua::createtable(L, static_cast<int>(sources.size()), 0);
    for (size_t i = 0; i < sources.size(); i++) {
        lua::pushlstring(L, sources[i].string());
        lua::rawseti(L, static_cast<int>(i + 1));
    }
    return 1;
}

/// @brief Set content sources list
static int l_set_content_sources(lua::State* L) {
    if (!lua::istable(L, 1)) {
        throw std::runtime_error("table expected as argument 1");
    }
    int len = lua::objlen(L, 1);
    std::vector<io::path> sources;
    for (int i = 0; i < len; i++) {
        lua::rawgeti(L, i + 1);
        sources.emplace_back(std::string(lua::require_lstring(L, -1)));
        lua::pop(L);
    }
    engine->getContentControl().setContentSources(std::move(sources));
    return 0;
}

/// @brief Reset content sources to defaults
static int l_reset_content_sources(lua::State* L) {
    engine->getContentControl().resetContentSources();
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

/// @brief Open folder in file explorer
static int l_open_folder(lua::State* L) {
    platform::open_folder(io::resolve(lua::require_string(L, 1)));
    return 0;
}

/// @brief Open URL in default browser with confirmation dialog
static int l_open_url(lua::State* L) {
    auto url = lua::require_string(L, 1);

    std::wstring msg = langs::get(L"Are you sure you want to open the link:") +
                       L"\n" + util::str2wstr_utf8(url) +
                       std::wstring(L"?");

    auto menu = engine->getGUI().getMenu();

    guiutil::confirm(*engine, msg, [url, menu]() {
        platform::open_url(url);
        if (!menu->back()) {
            menu->reset();
        }
    });
    return 0;
}


/// @brief Bring application window to focus
static int l_focus(lua::State* L) {
    engine->getWindow().focus();
    return 0;
}

/// @brief Set window title 
static int l_set_title(lua::State* L) {
    auto title = lua::require_string(L, 1);
    engine->getWindow().setTitle(title);
    return 0;
}

/// @brief Quit the game
static int l_quit(lua::State*) {
    engine->quit();
    return 0;
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
    controller->processBeforeQuit();
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

/// @brief Get engine version
/// @return major, minor 
static int l_get_version(lua::State* L) {
    return lua::pushvec_stack(
        L, glm::vec2(ENGINE_VERSION_MAJOR, ENGINE_VERSION_MINOR)
    );
}

/// @brief Create in-memory named IO device
static int l_create_memory_device(lua::State* L) {
    std::string name = lua::require_string(L, 1);
    if (io::get_device(name)) {
        throw std::runtime_error(
            "entry-point '" + name + "' is already used"
        );
    }
    if (name.find(':') != std::string::npos) {
        throw std::runtime_error("invalid entry point name");
    }
    
    io::set_device(name, std::make_unique<io::MemoryDevice>());
    return 0;
}

/// @brief Start a new engine instance with debugging server
static int l_start_debug_instance(lua::State* L) {
    if (!engine->getProject().permissions.has(Permissions::DEBUGGING)) {
        throw std::runtime_error("project has no debugging permission");
    }

    int port = lua::tointeger(L, 1);
    if (port == 0) {
        auto network = engine->getNetwork();
        if (network == nullptr) {
            throw std::runtime_error("project has no network permission");
        }
        port = network->findFreePort();
        if (port == -1) {
            throw std::runtime_error("could not find free port");
        }
    }
    auto projectPath = lua::isstring(L, 2) ? lua::require_lstring(L, 2) : "";
    const auto& paths = engine->getPaths();

    std::vector<std::string> args {
        "--res", paths.getResourcesFolder().u8string(),
        "--dir", paths.getUserFilesFolder().u8string(),
        "--dbg-server",  "tcp:" + std::to_string(port),
    };
    if (!projectPath.empty()) {
        args.emplace_back("--project");
        args.emplace_back(io::resolve(std::string(projectPath)).string());
    }

    platform::new_engine_instance(std::move(args));
    return lua::pushinteger(L, port);
}

const luaL_Reg applib[] = {
    /// content
    {"is_content_loaded", lua::wrap<l_is_content_loaded>},
    {"load_content", lua::wrap<l_load_content>},
    {"reset_content", lua::wrap<l_reset_content>},
    {"reconfig_packs", lua::wrap<l_reconfig_packs>},
    {"get_content_sources", lua::wrap<l_get_content_sources>},
    {"set_content_sources", lua::wrap<l_set_content_sources>},
    {"reset_content_sources", lua::wrap<l_reset_content_sources>},
    /// settings
    {"get_setting", lua::wrap<l_get_setting>},
    {"set_setting", lua::wrap<l_set_setting>},
    {"str_setting", lua::wrap<l_str_setting>},
    {"get_setting_info", lua::wrap<l_get_setting_info>},
    /// system applications
    {"open_folder", lua::wrap<l_open_folder>},
    {"open_url", lua::wrap<l_open_url>},
    /// window
    {"focus", lua::wrap<l_focus>},
    {"set_title", lua::wrap<l_set_title>},
    {"quit", lua::wrap<l_quit>},
    /// world
    {"new_world", lua::wrap<l_new_world>},
    {"open_world", lua::wrap<l_open_world>},
    {"reopen_world", lua::wrap<l_reopen_world>},
    {"save_world", lua::wrap<l_save_world>},
    {"close_world", lua::wrap<l_close_world>},
    {"delete_world", lua::wrap<l_delete_world>},
    /// other
    {"get_version", lua::wrap<l_get_version>},
    {"create_memory_device", lua::wrap<l_create_memory_device>},
    {"start_debug_instance", lua::wrap<l_start_debug_instance>},
    {nullptr, nullptr}
};
