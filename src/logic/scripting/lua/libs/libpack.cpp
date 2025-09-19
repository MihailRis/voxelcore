#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <set>

#include "assets/AssetsLoader.hpp"
#include "content/Content.hpp"
#include "content/ContentControl.hpp"
#include "content/PacksManager.hpp"
#include "engine/Engine.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/gui_util.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "frontend/locale.hpp"
#include "world/files/WorldFiles.hpp"
#include "io/engine_paths.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "api_lua.hpp"

using namespace scripting;

static int l_pack_get_folder(lua::State* L) {
    std::string packName = lua::tostring(L, 1);
    auto packs = content_control->getAllContentPacks();

    for (auto& pack : packs) {
        if (pack.id == packName) {
            return lua::pushstring(L, pack.folder.string() + "/");
        }
    }
    return lua::pushstring(L, "");
}

/// @brief pack.get_installed() -> array<string>
static int l_pack_get_installed(lua::State* L) {
    auto& packs = content_control->getContentPacks();
    lua::createtable(L, packs.size(), 0);
    for (size_t i = 0; i < packs.size(); i++) {
        lua::pushstring(L, packs[i].id);
        lua::rawseti(L, i + 1);
    }
    return 1;
}

/// @brief pack.get_available() -> array<string>
static int l_pack_get_available(lua::State* L) {
    auto& manager = content_control->scan();
    const auto& installed = content_control->getContentPacks();
    for (auto& pack : installed) {
        manager.exclude(pack.id);
    }
    auto names = manager.getAllNames();

    lua::createtable(L, names.size(), 0);
    for (size_t i = 0; i < names.size(); i++) {
        lua::pushstring(L, names[i]);
        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_pack_get_info(
    lua::State* L, const ContentPack& pack, const Content* content
) {
    lua::createtable(L, 0, 6);

    lua::pushstring(L, pack.id);
    lua::setfield(L, "id");

    lua::pushstring(L, pack.title);
    lua::setfield(L, "title");

    lua::pushstring(L, pack.creator);
    lua::setfield(L, "creator");

    lua::pushstring(L, pack.description);
    lua::setfield(L, "description");

    lua::pushstring(L, pack.version);
    lua::setfield(L, "version");

    lua::pushstring(L, pack.folder.string());
    lua::setfield(L, "path");

    if (!engine->isHeadless()) {
        auto assets = engine->getAssets();
        std::string icon = pack.id + ".icon";
        if (!AssetsLoader::loadExternalTexture(
                assets, icon, {pack.folder / "icon.png"}
            )) {
            icon = "gui/no_icon";
        }
        lua::pushstring(L, icon);
        lua::setfield(L, "icon");
    }

    if (!pack.dependencies.empty()) {
        lua::createtable(L, pack.dependencies.size(), 0);
        for (size_t i = 0; i < pack.dependencies.size(); i++) {
            auto& dpack = pack.dependencies[i];
            std::string prefix;
            switch (dpack.level) {
                case DependencyLevel::REQUIRED:
                    prefix = "!";
                    break;
                case DependencyLevel::OPTIONAL:
                    prefix = "?";
                    break;
                case DependencyLevel::WEAK:
                    prefix = "~";
                    break;
                default:
                    throw std::runtime_error("");
            }

            lua::pushfstring(L, "%s%s@%s%s", prefix.c_str(), dpack.id.c_str(), dpack.op.c_str(), dpack.version.c_str());
            lua::rawseti(L, i + 1);
        }
        lua::setfield(L, "dependencies");
    }

    auto runtime = content ? content->getPackRuntime(pack.id) : nullptr;
    if (runtime) {
        lua::pushboolean(L, runtime->getStats().hasSavingContent());
    } else {
        auto stats = pack.loadStats();
        lua::pushboolean(
            L, stats.has_value() ? stats->hasSavingContent() : false
        );
    }
    lua::setfield(L, "has_indices");
    return 1;
}

static int pack_get_infos(lua::State* L) {
    std::set<std::string> ids;
    size_t len = lua::objlen(L, 1);
    for (size_t i = 1; i <= len; i++) {
        lua::rawgeti(L, i);
        ids.insert(lua::require_string(L, -1));
        lua::pop(L, 1);
    }
    std::unordered_map<std::string, ContentPack> packs;
    const auto& loadedPacks = content_control->getContentPacks();
    for (const auto& pack : loadedPacks) {
        if (ids.find(pack.id) != ids.end()) {
            packs[pack.id] = pack;
            ids.erase(pack.id);
        }
    }
    if (!ids.empty()) {
        auto& manager = content_control->scan();
        auto vec =
            manager.getAll(std::vector<std::string>(ids.begin(), ids.end()));
        for (const auto& pack : vec) {
            packs[pack.id] = pack;
        }
    }
    lua::createtable(L, 0, packs.size());
    for (const auto& [id, pack] : packs) {
        l_pack_get_info(L, pack, content);
        lua::setfield(L, id);
    }
    return 1;
}

/// @brief pack.get_info(packid: str) -> {
///     title: str,
///     creator: str,
///     description: str,
///     version: str,
///     [optional] has_indices: bool
/// } or nil
static int l_pack_get_info(lua::State* L) {
    if (lua::istable(L, 1)) {
        return pack_get_infos(L);
    } else if (!lua::isstring(L, 1)) {
        throw std::runtime_error("string or table expected");
    }
    auto packid = lua::tostring(L, 1);

    auto& packs = content_control->getContentPacks();
    auto found =
        std::find_if(packs.begin(), packs.end(), [packid](const auto& pack) {
            return pack.id == packid;
        });
    if (found == packs.end()) {
        auto& manager = content_control->scan();
        auto vec = manager.getAll({packid});
        if (!vec.empty()) {
            return l_pack_get_info(L, vec[0], content);
        }
        return 0;
    }
    const auto& pack = *found;
    return l_pack_get_info(L, pack, content);
}

static int l_pack_get_base_packs(lua::State* L) {
    auto& packs = content_control->getBasePacks();
    lua::createtable(L, packs.size(), 0);
    for (size_t i = 0; i < packs.size(); i++) {
        lua::pushstring(L, packs[i]);
        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_pack_assemble(lua::State* L) {
    if (!lua::istable(L, 1)) {
        throw std::runtime_error("table expected");
    }
    std::vector<std::string> ids;
    size_t len = lua::objlen(L, 1);
    for (size_t i = 1; i <= len; i++) {
        lua::rawgeti(L, i);
        ids.push_back(lua::require_string(L, -1));
        lua::pop(L);
    }
    auto& manager = content_control->scan();
    try {
        ids = manager.assemble(ids);
    } catch (const contentpack_error& err) {
        throw std::runtime_error(
            std::string(err.what()) + " [" + err.getPackId() + "]"
        );
    }

    lua::createtable(L, ids.size(), 0);
    for (size_t i = 0; i < ids.size(); i++) {
        lua::pushstring(L, ids[i]);
        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_pack_request_writeable(lua::State* L) {
    auto packid = lua::require_string(L, 1);
    lua::pushvalue(L, 2);
    auto handler = lua::create_lambda_nothrow(L);

    auto str = langs::get(L"Grant %{0} pack modification permission?");
    util::replaceAll(str, L"%{0}", util::str2wstr_utf8(packid));
    guiutil::confirm(*engine, str, [packid, handler]() {
        handler({engine->getPaths().createWriteableDevice(packid)});
        engine->getGUI().getMenu()->reset();
    });
    return 0;
}

const luaL_Reg packlib[] = {
    {"get_folder", lua::wrap<l_pack_get_folder>},
    {"get_installed", lua::wrap<l_pack_get_installed>},
    {"get_available", lua::wrap<l_pack_get_available>},
    {"get_info", lua::wrap<l_pack_get_info>},
    {"get_base_packs", lua::wrap<l_pack_get_base_packs>},
    {"assemble", lua::wrap<l_pack_assemble>},
    {"request_writeable", lua::wrap<l_pack_request_writeable>},
    {NULL, NULL}
};
