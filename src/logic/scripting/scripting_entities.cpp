#include "scripting.hpp"

#include "lua/lua_engine.hpp"
#include "objects/Entities.hpp"
#include "objects/EntityDef.hpp"
#include "objects/Entity.hpp"
#include "objects/Player.hpp"
#include "util/stringutil.hpp"

using namespace scripting;

static inline const std::string STDCOMP = "stdcomp";

[[nodiscard]] static scriptenv create_component_environment(
    const scriptenv& parent, int entityIdx, const std::string& name
) {
    auto L = lua::get_main_state();
    int id = lua::create_environment(L, *parent);

    lua::pushvalue(L, entityIdx);

    lua::pushenv(L, id);

    lua::pushvalue(L, -1);
    lua::setfield(L, "this");

    lua::pushvalue(L, -2);
    lua::setfield(L, "entity");

    lua::pop(L);
    if (lua::getfield(L, "components")) {
        lua::pushenv(L, id);
        lua::setfield(L, name);
        lua::pop(L);
    }
    lua::pop(L);

    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

dv::value scripting::get_component_value(
    const scriptenv& env, const std::string& name
) {
    auto L = lua::get_main_state();
    lua::pushenv(L, *env);
    if (lua::getfield(L, name)) {
        return lua::tovalue(L, -1);
    }
    return nullptr;
}

static void create_component(
    lua::State* L,
    int entityIdx,
    UserComponent& component,
    const dv::value& args,
    const dv::value& saved
) {
    lua::pushvalue(L, entityIdx);
    auto compenv = create_component_environment(
        get_root_environment(), -1, component.name
    );
    lua::get_from(L, lua::CHUNKS_TABLE, component.name, true);
    lua::pushenv(L, *compenv);

    if (args != nullptr) {
        std::string compfieldname = component.name;
        util::replaceAll(compfieldname, ":", "__");
        if (args.has(compfieldname)) {
            lua::pushvalue(L, args[compfieldname]);
        } else {
            lua::createtable(L, 0, 0);
        }
    } else if (component.params != nullptr) {
        lua::pushvalue(L, component.params);
    } else {
        lua::createtable(L, 0, 0);
    }
    lua::setfield(L, "ARGS");

    if (saved == nullptr) {
        lua::createtable(L, 0, 0);
    } else {
        if (saved.has(component.name)) {
            lua::pushvalue(L, saved[component.name]);
        } else {
            lua::createtable(L, 0, 0);
        }
    }
    lua::setfield(L, "SAVED_DATA");

    lua::setfenv(L);
    lua::call_nothrow(L, 0, 0);

    lua::pushenv(L, *compenv);
    auto& funcsset = component.funcsset;
    funcsset.on_grounded = lua::hasfield(L, "on_grounded");
    funcsset.on_fall = lua::hasfield(L, "on_fall");
    funcsset.on_despawn = lua::hasfield(L, "on_despawn");
    funcsset.on_sensor_enter = lua::hasfield(L, "on_sensor_enter");
    funcsset.on_sensor_exit = lua::hasfield(L, "on_sensor_exit");
    funcsset.on_save = lua::hasfield(L, "on_save");
    funcsset.on_aim_on = lua::hasfield(L, "on_aim_on");
    funcsset.on_aim_off = lua::hasfield(L, "on_aim_off");
    funcsset.on_attacked = lua::hasfield(L, "on_attacked");
    funcsset.on_used = lua::hasfield(L, "on_used");
    lua::pop(L, 2);

    component.env = compenv;
}

void scripting::on_entity_spawn(
    const EntityDef&,
    entityid_t eid,
    const std::vector<std::unique_ptr<UserComponent>>& components,
    const dv::value& args,
    const dv::value& saved
) {
    auto L = lua::get_main_state();
    lua::stackguard guard(L);
    lua::requireglobal(L, STDCOMP);
    if (lua::getfield(L, "new_Entity")) {
        lua::pushinteger(L, eid);
        lua::call(L, 1);
    }
    for (auto& component : components) {
        create_component(L, -1, *component, args, saved);
    }
}

static void process_entity_callback(
    const scriptenv& env,
    const std::string& name,
    std::function<int(lua::State*)> args
) {
    auto L = lua::get_main_state();
    lua::pushenv(L, *env);
    if (lua::hasfield(L, "__disabled")) {
        lua::pop(L);
        return;
    }
    if (lua::getfield(L, name)) {
        if (args) {
            lua::call_nothrow(L, args(L), 0);
        } else {
            lua::call_nothrow(L, 0, 0);
        }
    }
    lua::pop(L);
}

static void process_entity_callback(
    const Entity& entity,
    const std::string& name,
    bool EntityFuncsSet::*flag,
    std::function<int(lua::State*)> args
) {
    const auto& script = entity.getScripting();
    for (auto& component : script.components) {
        if (component->funcsset.*flag) {
            process_entity_callback(component->env, name, args);
        }
    }
}

void scripting::on_entity_despawn(const Entity& entity) {
    process_entity_callback(
        entity, "on_despawn", &EntityFuncsSet::on_despawn, nullptr
    );
    auto L = lua::get_main_state();
    lua::get_from(L, "stdcomp", "remove_Entity", true);
    lua::pushinteger(L, entity.getUID());
    lua::call(L, 1, 0);
}

void scripting::on_entity_grounded(const Entity& entity, float force) {
    process_entity_callback(
        entity,
        "on_grounded",
        &EntityFuncsSet::on_grounded,
        [force](auto L) { return lua::pushnumber(L, force); }
    );
}

void scripting::on_entity_fall(const Entity& entity) {
    process_entity_callback(
        entity, "on_fall", &EntityFuncsSet::on_fall, nullptr
    );
}

void scripting::on_entity_save(const Entity& entity) {
    process_entity_callback(
        entity, "on_save", &EntityFuncsSet::on_save, nullptr
    );
}

void scripting::on_sensor_enter(
    const Entity& entity, size_t index, entityid_t oid
) {
    process_entity_callback(
        entity,
        "on_sensor_enter",
        &EntityFuncsSet::on_sensor_enter,
        [index, oid](auto L) {
            lua::pushinteger(L, index);
            lua::pushinteger(L, oid);
            return 2;
        }
    );
}

void scripting::on_sensor_exit(
    const Entity& entity, size_t index, entityid_t oid
) {
    process_entity_callback(
        entity,
        "on_sensor_exit",
        &EntityFuncsSet::on_sensor_exit,
        [index, oid](auto L) {
            lua::pushinteger(L, index);
            lua::pushinteger(L, oid);
            return 2;
        }
    );
}

void scripting::on_aim_on(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_aim_on",
        &EntityFuncsSet::on_aim_on,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_aim_off(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_aim_off",
        &EntityFuncsSet::on_aim_off,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_attacked(
    const Entity& entity, Player* player, entityid_t attacker
) {
    process_entity_callback(
        entity,
        "on_attacked",
        &EntityFuncsSet::on_attacked,
        [player, attacker](auto L) {
            lua::pushinteger(L, attacker);
            lua::pushinteger(L, player->getId());
            return 2;
        }
    );
}

void scripting::on_entity_used(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_used",
        &EntityFuncsSet::on_used,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_entities_update(int tps, int parts, int part) {
    auto L = lua::get_main_state();
    lua::get_from(L, STDCOMP, "update", true);
    lua::pushinteger(L, tps);
    lua::pushinteger(L, parts);
    lua::pushinteger(L, part);
    lua::call_nothrow(L, 3, 0);
    lua::pop(L);
}

void scripting::on_entities_physics_update(float delta) {
    auto L = lua::get_main_state();
    lua::get_from(L, STDCOMP, "physics_update", true);
    lua::pushnumber(L, delta);
    lua::call_nothrow(L, 1, 0);
    lua::pop(L);
}

void scripting::on_entities_render(float delta) {
    auto L = lua::get_main_state();
    lua::get_from(L, STDCOMP, "render", true);
    lua::pushnumber(L, delta);
    lua::call_nothrow(L, 1, 0);
    lua::pop(L);
}
