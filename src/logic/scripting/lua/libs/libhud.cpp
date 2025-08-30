#include "libhud.hpp"

#include <glm/glm.hpp>

#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "frontend/UiDocument.hpp"
#include "frontend/hud.hpp"
#include "content/ContentControl.hpp"
#include "graphics/ui/elements/InventoryView.hpp"
#include "items/Inventories.hpp"
#include "logic/BlocksController.hpp"
#include "objects/Player.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/voxel.hpp"
#include "voxels/blocks_agent.hpp"
#include "world/Level.hpp"

using namespace scripting;

static int l_open_inventory(lua::State*) {
    if (!hud->isInventoryOpen()) {
        hud->openInventory();
    }
    return 0;
}

static int l_close_inventory(lua::State*) {
    if (hud->isInventoryOpen()) {
        hud->closeInventory();
    }
    return 0;
}

static int l_open(lua::State* L) {
    auto layoutid = lua::require_string(L, 1);
    bool playerInventory = !lua::toboolean(L, 2);
    auto invid = lua::tointeger(L, 3);

    auto assets = engine->getAssets();
    auto layout = assets->get<UiDocument>(layoutid);
    if (layout == nullptr) {
        throw std::runtime_error("there is no ui layout " + util::quote(layoutid));
    }
    return lua::pushinteger(L, hud->openInventory(
        layout,
        lua::isnoneornil(L, 3) ? nullptr : level->inventories->get(invid),
        playerInventory
    )->getId());
}

static int l_open_block(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    bool playerInventory = !lua::toboolean(L, 4);

    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    if (vox == nullptr) {
        throw std::runtime_error(
            "block does not exists at " + std::to_string(x) + " " +
            std::to_string(y) + " " + std::to_string(z)
        );
    }
    auto& def = content->getIndices()->blocks.require(vox->id);
    auto assets = engine->getAssets();
    auto layout = assets->get<UiDocument>(def.uiLayout);
    if (layout == nullptr) {
        throw std::runtime_error("block '" + def.name + "' has no ui layout");
    }

    auto id = blocks->createBlockInventory(x, y, z);
    hud->openInventory(
        glm::ivec3(x, y, z),
        layout,
        level->inventories->get(id),
        playerInventory
    );

    lua::pushinteger(L, id);
    lua::pushstring(L, def.uiLayout);
    return 2;
}

static int l_show_overlay(lua::State* L) {
    auto name = lua::require_string(L, 1);
    bool playerInventory = lua::toboolean(L, 2);

    auto assets = engine->getAssets();
    auto layout = assets->get<UiDocument>(name);
    if (layout == nullptr) {
        throw std::runtime_error("there is no ui layout " + util::quote(name));
    }
    auto args = lua::tovalue(L, 3);
    hud->showOverlay(layout, playerInventory, std::move(args));
    return 0;
}

static UiDocument* require_layout(const char* name) {
    auto assets = engine->getAssets();
    auto layout = assets->get<UiDocument>(name);
    if (layout == nullptr) {
        throw std::runtime_error(
            "layout '" + std::string(name) + "' is not found"
        );
    }
    return layout;
}

static int l_open_permanent(lua::State* L) {
    auto layout = require_layout(lua::require_string(L, 1));
    hud->openPermanent(layout);
    return 0;
}

static int l_close(lua::State* L) {
    auto layout = require_layout(lua::require_string(L, 1));
    hud->remove(layout->getRoot());
    return 0;
}

static int l_pause(lua::State*) {
    hud->setPause(true);
    return 0;
}

static int l_resume(lua::State*) {
    hud->setPause(false);
    return 0;
}

static int l_get_block_inventory(lua::State* L) {
    auto inventory = hud->getBlockInventory();
    if (inventory == nullptr) {
        return lua::pushinteger(L, 0);
    } else {
        return lua::pushinteger(L, inventory->getId());
    }
}

static int l_get_player(lua::State* L) {
    auto player = hud->getPlayer();
    return lua::pushinteger(L, player->getId());
}

static int l_is_paused(lua::State* L) {
    return lua::pushboolean(L, hud->isPause());
}

static int l_is_inventory_open(lua::State* L) {
    return lua::pushboolean(L, hud->isInventoryOpen());
}

static int l_is_content_access(lua::State* L) {
    return lua::pushboolean(L, hud->isContentAccess());
}

static int l_set_content_access(lua::State* L) {
    hud->setContentAccess(lua::toboolean(L, 1));
    return 0;
}

static int l_set_debug_cheats(lua::State* L) {
    hud->setDebugCheats(lua::toboolean(L, 1));
    return 0;
}

static int l_set_allow_pause(lua::State* L) {
    hud->setAllowPause(lua::toboolean(L, 1));
    return 0;
}

static int l_reload_script(lua::State* L) {
    auto packid = lua::require_string(L, 1);
    if (content == nullptr) {
        throw std::runtime_error("content is not initialized");
    }
    auto& writeableContent = *content_control->get();
    auto pack = writeableContent.getPackRuntime(packid);
    const auto& info = pack->getInfo();
    scripting::load_hud_script(
        pack->getEnvironment(),
        packid,
        info.folder / "scripts/hud.lua",
        pack->getId() + ":scripts/hud.lua"
    );
    return 0;
}

const luaL_Reg hudlib[] = {
    {"open_inventory", wrap_hud<l_open_inventory>},
    {"close_inventory", wrap_hud<l_close_inventory>},
    {"open", wrap_hud<l_open>},
    {"open_block", wrap_hud<l_open_block>},
    {"open_permanent", wrap_hud<l_open_permanent>},
    {"show_overlay", wrap_hud<l_show_overlay>},
    {"get_block_inventory", wrap_hud<l_get_block_inventory>},
    {"close", wrap_hud<l_close>},
    {"pause", wrap_hud<l_pause>},
    {"resume", wrap_hud<l_resume>},
    {"is_paused", wrap_hud<l_is_paused>},
    {"is_inventory_open", wrap_hud<l_is_inventory_open>},
    {"get_player", wrap_hud<l_get_player>},
    {"_is_content_access", wrap_hud<l_is_content_access>},
    {"_set_content_access", wrap_hud<l_set_content_access>},
    {"_set_debug_cheats", wrap_hud<l_set_debug_cheats>},
    {"set_allow_pause", wrap_hud<l_set_allow_pause>},
    {"reload_script", wrap_hud<l_reload_script>},
    {NULL, NULL}
};
