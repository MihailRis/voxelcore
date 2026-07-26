#include "libentity.hpp"

#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "engine/Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "objects/Entities.hpp"
#include "objects/EntityDef.hpp"
#include "objects/Entity.hpp"
#include "objects/Rigidbody.hpp"
#include "objects/Player.hpp"
#include "objects/rigging.hpp"
#include "physics/Hitbox.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Block.hpp"
#include "voxels/blocks_agent.hpp"
#include "window/Camera.hpp"

#include <set>

using namespace scripting;

static const EntityDef* require_entity_def(lua::State* L) {
    auto indices = content->getIndices();
    auto id = lua::tointeger(L, 1);
    return indices->entities.get(id);
}

static int l_exists(lua::State* L) {
    return lua::pushboolean(L, get_entity(L, 1).has_value());
}

static int l_def_index(lua::State* L) {
    auto name = lua::require_string(L, 1);
    return lua::pushinteger(L, content->entities.require(name).rt.id);
}

static int l_def_name(lua::State* L) {
    if (auto def = require_entity_def(L)) {
        return lua::pushstring(L, def->name);
    }
    return 0;
}

static int l_def_hitbox(lua::State* L) {
    if (auto def = require_entity_def(L)) {
        return lua::pushvec(L, def->hitbox);
    }
    return 0;
}

static int l_def_solid(lua::State* L) {
    if (auto def = require_entity_def(L)) {
        return lua::pushboolean(L, def->solid);
    }
    return 0;
}

static int l_defs_count(lua::State* L) {
    return lua::pushinteger(L, indices->entities.count());
}

static int l_get_def(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushinteger(L, entity->getDef().rt.id);
    }
    return 0;
}

static int l_spawn(lua::State* L) {
    auto level = controller->getLevel();
    auto defname = lua::tostring(L, 1);
    auto& def = content->entities.require(defname);
    auto pos = lua::tovec3(L, 2);
    dv::value args = nullptr;
    if (lua::gettop(L) > 2) {
        args = lua::tovalue(L, 3);
    }
    entityid_t id = level->entities->spawn(def, pos, std::move(args));
    lua::get_from(L, "entities", "get", true);
    lua::pushinteger(L, id);
    return lua::call_nothrow(L, 1);
}

static int l_despawn(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->destroy();
    }
    return 0;
}

static int l_get_skeleton(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        auto skeleton = entity->getSkeleton();
        if (skeleton == nullptr) {
            return 0;
        }
        return lua::pushstring(L, skeleton->config->getName());
    }
    return 0;
}

static int l_get_player(lua::State* L) {
    entityid_t eid = lua::touinteger(L, 1);
    auto level = controller->getLevel();
    if (auto entity = level->entities->get(eid)) {
        return lua::pushinteger(L, entity->getPlayer());
    }
    return 0;
}

static int l_set_skeleton(lua::State* L) {
    auto assets = engine->getAssets();
    if (assets == nullptr) {
        return 0;
    }
    if (auto entity = get_entity(L, 1)) {
        std::string skeletonName = lua::require_string(L, 2);
        auto rigConfig = assets->getShared<rigging::SkeletonConfig>(skeletonName);
        if (rigConfig == nullptr) {
            throw std::runtime_error(
                "skeleton not found '" + skeletonName + "'"
            );
        }
        entity->setRig(std::move(rigConfig));
    }
    return 0;
}

static int l_get_all_in_box(lua::State* L) {
    auto pos = lua::tovec<3>(L, 1);
    auto size = lua::tovec<3>(L, 2);
    auto found = level->entities->getAllInside(AABB(pos, pos + size));
    lua::createtable(L, found.size(), 0);
    for (size_t i = 0; i < found.size(); i++) {
        const auto& entity = found[i];
        lua::pushinteger(L, entity.getUID());
        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_get_all_in_radius(lua::State* L) {
    auto pos = lua::tovec<3>(L, 1);
    auto radius = lua::tonumber(L, 2);
    auto found = level->entities->getAllInRadius(pos, radius);
    lua::createtable(L, found.size(), 0);
    for (size_t i = 0; i < found.size(); i++) {
        const auto& entity = found[i];
        lua::pushinteger(L, entity.getUID());
        lua::rawseti(L, i + 1);
    }
    return 1;
}

static int l_push_ray(
    lua::State* L,
    const std::optional<Entities::RaycastResult>& ray,
    blockid_t block,
    const glm::vec3& start,
    const glm::vec3& dir,
    const glm::vec3& end,
    const glm::ivec3& iend,
    const glm::ivec3& normal
) {
    if (!ray.has_value() && block == BLOCK_VOID) {
        return 0;
    }
    if (lua::gettop(L) >= 5 && lua::istable(L, 5)) {
        lua::pushvalue(L, 5);
    } else {
        lua::createtable(L, 0, 6);
    }

    lua::pushvec3(L, ray ? (start + dir * ray->distance) : end);
    lua::setfield(L, "endpoint");

    lua::pushvec3(L, ray ? ray->normal : normal);
    lua::setfield(L, "normal");

    lua::pushnumber(L, glm::distance(start, end));
    lua::setfield(L, "length");

    lua::pushvec3(L, iend);
    lua::setfield(L, "iendpoint");

    lua::pushinteger(L, block);
    lua::setfield(L, "block");

    if (ray.has_value()) {
        lua::pushinteger(L, ray->entity);
        lua::setfield(L, "entity");
    } else if (block != BLOCK_VOID) {
        lua::pushinteger(L, block);
        lua::setfield(L, "block");
    }
    return 1;
}

static int l_raycast(
    lua::State* L,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDistance,
    const blocks_agent::RaycastSettings& voxelRaycast,
    const Entities::RaycastSettings& entityRaycast,
    bool entities
) {
    glm::vec3 end;
    glm::ivec3 normal;
    glm::ivec3 iend;

    blockid_t block = BLOCK_VOID;

    if (auto voxel = blocks_agent::raycast(
            *level->chunks,
            start,
            dir,
            maxDistance,
            end,
            normal,
            iend,
            voxelRaycast
        )) {
        maxDistance = glm::distance(start, end);
        block = voxel->id;
    }
    auto ray =
        entities
            ? level->entities->rayCast(start, dir, maxDistance, entityRaycast)
            : std::nullopt;
    return l_push_ray(L, ray, block, start, dir, end, iend, normal);
}

static void load_blocks_filter(lua::State* L, int idx, std::set<blockid_t>& dst) {
    int count = lua::objlen(L, idx);
    for (int i = 0; i < count; i++) {
        lua::rawgeti(L, i + 1, idx);
        if (lua::isnumber(L, -1)) {
            dst.insert(lua::touinteger(L, -1));
        } else {
            auto blockName = std::string(lua::tostring(L, -1));
            const Block* block = content->blocks.find(blockName);
            if (block != nullptr) {
                dst.insert(block->rt.id);
            }
        }
        lua::pop(L);
    }
}

static void load_entities_filter(lua::State* L, int idx, std::set<entitydefid_t>& dst) {
    int count = lua::objlen(L, idx);
    for (int i = 0; i < count; i++) {
        lua::rawgeti(L, i + 1, idx);
        if (lua::isnumber(L, -1)) {
            dst.insert(lua::touinteger(L, -1));
        } else {
            auto entityName = std::string(lua::tostring(L, -1));
            const EntityDef* entity = content->entities.find(entityName);
            if (entity != nullptr) {
                dst.insert(entity->rt.id);
            }
        }
        lua::pop(L);
    }
}

static int l_world_raycast(lua::State* L) {
    blocks_agent::RaycastSettings blocksRaycast {};
    Entities::RaycastSettings entityRaycast {};

    std::set<blockid_t> filteredBlocks {};
    std::set<entitydefid_t> filteredEntities {};

    glm::vec3 start, dir;
    float maxDistance;
    bool entities = true;

    if (!lua::istable(L, 1)) {
        throw std::runtime_error("invalid function overload");
    }

    lua::requirefield(L, "start", 1);
    start = lua::tovec3(L, -1);

    lua::requirefield(L, "dir", 1);
    dir = lua::tovec3(L, -1);

    lua::requirefield(L, "distance", 1);
    maxDistance = lua::tonumber(L, -1);

    lua::pop(L, 3);

    if (lua::getfield(L, "entities", 1)) {
        entities = lua::toboolean(L, -1);
        lua::pop(L);
    }

    if (lua::getfield(L, "ignore_uid", 1)) {
        entityRaycast.ignoredUid = lua::tointeger(L, -1);
        lua::pop(L);
    }
    if (lua::getfield(L, "filter_entities")) {
        load_entities_filter(L, -1, filteredEntities);
        blocksRaycast.filter = &filteredEntities;
        lua::pop(L);
    }
    if (lua::getfield(L, "entities_exclusion")) {
        entityRaycast.entityFilterExcludeMode = lua::toboolean(L, -1);
        lua::pop(L);
    }
    if (lua::getfield(L, "filter_blocks", 1)) {
        load_blocks_filter(L, -1, filteredBlocks);
        blocksRaycast.filter = &filteredBlocks;
        lua::pop(L);
    }
    if (lua::getfield(L, "blocks_exclusion")) {
        blocksRaycast.blocksFilterExcludeMode = lua::toboolean(L, -1);
        lua::pop(L);
    }
    if (lua::getfield(L, "nonselect_blocks", 1)) {
        blocksRaycast.includeNonSelectable = lua::toboolean(L, -1);
        lua::pop(L);
    }
    return l_raycast(
        L, start, dir, maxDistance, blocksRaycast, entityRaycast, entities
    );
}

static int l_raycast(lua::State* L) {
    blocks_agent::RaycastSettings blocksRaycast {};
    Entities::RaycastSettings entityRaycast {};
    std::set<blockid_t> filteredBlocks {};

    const int luaStackSize = lua::gettop(L);
    glm::vec3 start = lua::tovec<3>(L, 1);
    glm::vec3 dir = lua::tovec<3>(L, 2);
    float maxDistance = lua::tonumber(L, 3);

    entityRaycast.ignoredUid = lua::tointeger(L, 4);
    if (luaStackSize >= 6) {
        if (lua::istable(L, 6)) {
            load_blocks_filter(L, 6, filteredBlocks);
            blocksRaycast.filter = &filteredBlocks;
        } else {
            throw std::runtime_error("table expected for filter");
        }
    }
    if (luaStackSize >= 7) {
        blocksRaycast.includeNonSelectable = lua::toboolean(L, 6);
    }
    return l_raycast(
        L, start, dir, maxDistance, blocksRaycast, entityRaycast, true
    );
}

static int l_reload_component(lua::State* L) {
    std::string name = lua::require_string(L, 1);
    size_t pos = name.find(':');
    if (pos == std::string::npos) {
        throw std::runtime_error("missing entry point");
    }
    auto filename = name.substr(0, pos + 1) + "scripts/components/" +
                    name.substr(pos + 1) + ".lua";
    auto prefix = name.substr(0, pos);
    auto runtime = content->getPackRuntime(prefix);
    if (runtime == nullptr) {
        throw std::runtime_error("pack '" + prefix + "' content is not loaded");
    }
    scripting::load_entity_component(
        runtime->getEnvironment(), name, filename, filename
    );
    return 0;
}

const luaL_Reg entitylib[] = {
    {"exists", lua::wrap<l_exists>},
    {"def_index", lua::wrap<l_def_index>},
    {"def_name", lua::wrap<l_def_name>},
    {"def_hitbox", lua::wrap<l_def_hitbox>},
    {"def_solid", lua::wrap<l_def_solid>},
    {"get_def", lua::wrap<l_get_def>},
    {"defs_count", lua::wrap<l_defs_count>},
    {"spawn", lua::wrap<l_spawn>},
    {"despawn", lua::wrap<l_despawn>},
    {"get_skeleton", lua::wrap<l_get_skeleton>},
    {"set_skeleton", lua::wrap<l_set_skeleton>},
    {"get_player", lua::wrap<l_get_player>},
    {"get_all_in_box", lua::wrap<l_get_all_in_box>},
    {"get_all_in_radius", lua::wrap<l_get_all_in_radius>},
    {"raycast", lua::wrap<l_raycast>},
    {"__world_raycast", lua::wrap<l_world_raycast>},
    {"reload_component", lua::wrap<l_reload_component>},
    {nullptr, nullptr}
};
