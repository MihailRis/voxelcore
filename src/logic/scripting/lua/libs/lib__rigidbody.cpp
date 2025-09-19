#define VC_ENABLE_REFLECTION

#include "util/stringutil.hpp"
#include "libentity.hpp"

static int l_get_vel(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushvec3(L, entity->getRigidbody().hitbox.velocity);
    }
    return 0;
}

static int l_set_vel(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().hitbox.velocity = lua::tovec3(L, 2);
    }
    return 0;
}

static int l_is_enabled(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushboolean(L, entity->getRigidbody().enabled);
    }
    return 0;
}

static int l_set_enabled(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().enabled = lua::toboolean(L, 2);
    }
    return 0;
}

static int l_get_size(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushvec3(L, entity->getRigidbody().hitbox.halfsize * 2.0f);
    }
    return 0;
}

static int l_set_size(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().hitbox.halfsize = lua::tovec3(L, 2) * 0.5f;
    }
    return 0;
}

static int l_get_gravity_scale(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushnumber(L, entity->getRigidbody().hitbox.gravityScale);
    }
    return 0;
}

static int l_set_gravity_scale(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().hitbox.gravityScale = lua::tovec3(L, 2).y;
    }
    return 0;
}

static int l_is_vdamping(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushboolean(
            L, entity->getRigidbody().hitbox.verticalDamping > 0.0
        );
    }
    return 0;
}

static int l_get_vdamping(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushnumber(
            L, entity->getRigidbody().hitbox.verticalDamping
        );
    }
    return 0;
}

static int l_set_vdamping(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        if (lua::isboolean(L, 2)) {
            entity->getRigidbody().hitbox.verticalDamping = lua::toboolean(L, 2);
        } else {
            entity->getRigidbody().hitbox.verticalDamping = lua::tonumber(L, 2);
        }
    }
    return 0;
}

static int l_is_grounded(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushboolean(L, entity->getRigidbody().hitbox.grounded);
    }
    return 0;
}

static int l_is_crouching(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushboolean(L, entity->getRigidbody().hitbox.crouching);
    }
    return 0;
}

static int l_set_crouching(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().hitbox.crouching = lua::toboolean(L, 2);
    }
    return 0;
}

static int l_get_body_type(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushlstring(
            L, BodyTypeMeta.getName(entity->getRigidbody().hitbox.type)
        );
    }
    return 0;
}

static int l_set_body_type(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        if (!BodyTypeMeta.getItem(
                lua::tostring(L, 2), entity->getRigidbody().hitbox.type
            )) {
            throw std::runtime_error(
                "unknown body type " + util::quote(lua::tostring(L, 2))
            );
        }
    }
    return 0;
}

static int l_get_linear_damping(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushnumber(L, entity->getRigidbody().hitbox.linearDamping);
    }
    return 0;
}

static int l_set_linear_damping(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        entity->getRigidbody().hitbox.linearDamping = lua::tonumber(L, 2);
    }
    return 0;
}

const luaL_Reg rigidbodylib[] = {
    {"is_enabled", lua::wrap<l_is_enabled>},
    {"set_enabled", lua::wrap<l_set_enabled>},
    {"get_vel", lua::wrap<l_get_vel>},
    {"set_vel", lua::wrap<l_set_vel>},
    {"get_size", lua::wrap<l_get_size>},
    {"set_size", lua::wrap<l_set_size>},
    {"get_gravity_scale", lua::wrap<l_get_gravity_scale>},
    {"set_gravity_scale", lua::wrap<l_set_gravity_scale>},
    {"get_linear_damping", lua::wrap<l_get_linear_damping>},
    {"set_linear_damping", lua::wrap<l_set_linear_damping>},
    {"is_vdamping", lua::wrap<l_is_vdamping>},
    {"get_vdamping", lua::wrap<l_get_vdamping>},
    {"set_vdamping", lua::wrap<l_set_vdamping>},
    {"is_grounded", lua::wrap<l_is_grounded>},
    {"is_crouching", lua::wrap<l_is_crouching>},
    {"set_crouching", lua::wrap<l_set_crouching>},
    {"get_body_type", lua::wrap<l_get_body_type>},
    {"set_body_type", lua::wrap<l_set_body_type>},
    {NULL, NULL}};
