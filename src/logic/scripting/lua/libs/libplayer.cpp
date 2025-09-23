#include <algorithm>
#include <glm/glm.hpp>

#include "items/Inventory.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "objects/Entity.hpp"
#include "physics/Hitbox.hpp"
#include "window/Camera.hpp"
#include "world/Level.hpp"
#include "libentity.hpp"

using namespace scripting;

inline Player* get_player(lua::State* L, int idx) {
    return level->players->get(lua::tointeger(L, idx));
}

static int l_get_pos(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushvec_stack(L, player->getPosition());
    }
    return 0;
}

static int l_set_pos(lua::State* L) {
    auto player = get_player(L, 1);
    if (!player) {
        return 0;
    }
    auto x = lua::tonumber(L, 2);
    auto y = lua::tonumber(L, 3);
    auto z = lua::tonumber(L, 4);
    player->teleport(glm::vec3(x, y, z));
    return 0;
}

static int l_get_vel(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        if (auto hitbox = player->getHitbox()) {
            return lua::pushvec_stack(L, hitbox->velocity);
        }
    }
    return 0;
}

static int l_set_vel(lua::State* L) {
    auto player = get_player(L, 1);
    if (!player) {
        return 0;
    }
    auto x = lua::tonumber(L, 2);
    auto y = lua::tonumber(L, 3);
    auto z = lua::tonumber(L, 4);
    
    if (auto hitbox = player->getHitbox()) {
        hitbox->velocity = glm::vec3(x, y, z);
    }
    return 0;
}

static int l_get_rot(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushvec_stack(L, player->getRotation(lua::toboolean(L, 2)));
    }
    return 0;
}

static int l_set_rot(lua::State* L) {
    auto player = get_player(L, 1);
    if (!player) {
        return 0;
    }
    glm::vec3 rotation = player->getRotation();

    auto x = lua::tonumber(L, 2);
    auto y = lua::tonumber(L, 3);
    auto z = rotation.z;
    if (lua::isnumber(L, 4)) {
        z = lua::tonumber(L, 4);
    }
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
    player->setRotation(rotation);
    return 0;
}

static int l_get_dir(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushvec3(L, player->fpCamera->front);
    }
    return 0;
}

static int l_get_inv(lua::State* L) {
    auto player = get_player(L, 1);
    if (!player) {
        return 0;
    }
    lua::pushinteger(L, player->getInventory()->getId());
    lua::pushinteger(L, player->getChosenSlot());
    return 2;
}

static int l_set_selected_slot(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setChosenSlot(lua::tointeger(L, 2) % 10);
    }
    return 0;
}

static int l_is_flight(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isFlight());
    }
    return 0;
}

static int l_set_flight(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setFlight(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_is_noclip(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isNoclip());
    }
    return 0;
}

static int l_set_noclip(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setNoclip(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_is_infinite_items(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isInfiniteItems());
    }
    return 0;
}

static int l_set_infinite_items(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setInfiniteItems(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_is_instant_destruction(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isInstantDestruction());
    }
    return 0;
}

static int l_set_instant_destruction(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setInstantDestruction(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_is_loading_chunks(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isLoadingChunks());
    }
    return 0;
}

static int l_set_loading_chunks(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setLoadingChunks(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_get_selected_block(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        if (player->selection.vox.id == BLOCK_VOID) {
            return 0;
        }
        return lua::pushivec_stack(L, player->selection.position);
    }
    return 0;
}

static int l_get_selected_entity(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        if (auto eid = player->getSelectedEntity()) {
            return lua::pushinteger(L, eid);
        }
    }
    return 0;
}

static int l_get_spawnpoint(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushvec_stack(L, player->getSpawnPoint());
    }
    return 0;
}

static int l_set_spawnpoint(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        auto x = lua::tonumber(L, 2);
        auto y = lua::tonumber(L, 3);
        auto z = lua::tonumber(L, 4);
        player->setSpawnPoint(glm::vec3(x, y, z));
    }
    return 0;
}

static int l_get_entity(lua::State* L) {
    auto player = get_player(L, 1);
    if (player == nullptr) {
        return 0;
    }
    return lua::pushinteger(L, player->getEntity());
}

static int l_set_entity(lua::State* L) {
    auto player = get_player(L, 1);
    if (player == nullptr) {
        return 0;
    }
    if (lua::isnumber(L, 2)) {
        player->setEntity(lua::tointeger(L, 2));
    } else if (auto entity = get_entity(L, 2)) {
        player->setEntity(entity->getUID());
    }
    return 0;
}

static int l_get_camera(lua::State* L) {
    auto player = get_player(L, 1);
    if (player == nullptr) {
        return 0;
    }
    auto found = std::find(
        level->cameras.begin(), level->cameras.end(), player->currentCamera
    );
    if (found == level->cameras.end()) {
        return 0;
    }
    return lua::pushinteger(L, found - level->cameras.begin());
}

static int l_set_camera(lua::State* L) {
    auto player = get_player(L, 1);
    if (player == nullptr) {
        return 0;
    }
    size_t index = lua::tointeger(L, 2);
    player->currentCamera = level->cameras.at(index);
    return 0;
}

static int l_get_name(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushstring(L, player->getName());
    }
    return 0;
}

static int l_set_name(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setName(lua::require_string(L, 2));
    }
    return 0;
}

static int l_create(lua::State* L) {
    int64_t playerId = Players::NONE;
    if (lua::gettop(L) >= 2) {
        playerId = lua::tointeger(L, 2);
    }
    auto player = level->players->create(playerId);
    player->setName(lua::require_string(L, 1));
    return lua::pushinteger(L, player->getId());
}

static int l_delete(lua::State* L) {
    auto id = lua::tointeger(L, 1);
    level->players->suspend(id);
    level->players->remove(id);
    return 0;
}

static int l_is_suspended(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        return lua::pushboolean(L, player->isSuspended());
    }
    return 0;
}

static int l_set_suspended(lua::State* L) {
    if (auto player = get_player(L, 1)) {
        player->setSuspended(lua::toboolean(L, 2));
    }
    return 0;
}

const luaL_Reg playerlib[] = {
    {"get_pos", lua::wrap<l_get_pos>},
    {"set_pos", lua::wrap<l_set_pos>},
    {"get_vel", lua::wrap<l_get_vel>},
    {"set_vel", lua::wrap<l_set_vel>},
    {"get_rot", lua::wrap<l_get_rot>},
    {"set_rot", lua::wrap<l_set_rot>},
    {"get_dir", lua::wrap<l_get_dir>},
    {"get_inventory", lua::wrap<l_get_inv>},
    {"is_suspended", lua::wrap<l_is_suspended>},
    {"set_suspended", lua::wrap<l_set_suspended>},
    {"is_flight", lua::wrap<l_is_flight>},
    {"set_flight", lua::wrap<l_set_flight>},
    {"is_noclip", lua::wrap<l_is_noclip>},
    {"set_noclip", lua::wrap<l_set_noclip>},
    {"is_infinite_items", lua::wrap<l_is_infinite_items>},
    {"set_infinite_items", lua::wrap<l_set_infinite_items>},
    {"is_instant_destruction", lua::wrap<l_is_instant_destruction>},
    {"set_instant_destruction", lua::wrap<l_set_instant_destruction>},
    {"is_loading_chunks", lua::wrap<l_is_loading_chunks>},
    {"set_loading_chunks", lua::wrap<l_set_loading_chunks>},
    {"set_selected_slot", lua::wrap<l_set_selected_slot>},
    {"get_selected_block", lua::wrap<l_get_selected_block>},
    {"get_selected_entity", lua::wrap<l_get_selected_entity>},
    {"set_spawnpoint", lua::wrap<l_set_spawnpoint>},
    {"get_spawnpoint", lua::wrap<l_get_spawnpoint>},
    {"get_entity", lua::wrap<l_get_entity>},
    {"set_entity", lua::wrap<l_set_entity>},
    {"get_camera", lua::wrap<l_get_camera>},
    {"set_camera", lua::wrap<l_set_camera>},
    {"get_name", lua::wrap<l_get_name>},
    {"set_name", lua::wrap<l_set_name>},
    {"create", lua::wrap<l_create>},
    {"delete", lua::wrap<l_delete>},
    {NULL, NULL}
};
