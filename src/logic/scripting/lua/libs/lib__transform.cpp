#include "libentity.hpp"

#include "maths/util.hpp"

static void check_valid(const glm::vec3& vec) {
    if (util::is_nan_or_inf(vec)) {
        throw std::invalid_argument(
            "invalid vector: " + std::to_string(vec.x) + ", " +
            std::to_string(vec.y) + ", " + std::to_string(vec.z)
        );
    }
}

static void check_valid(const glm::mat3& vec) {
    if (util::is_nan_or_inf(vec)) {
        throw std::invalid_argument(
            "invalid matrix (contains nan or inf values)"
        );
    }
}

static int l_get_pos(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushvec3(L, entity->getTransform().pos);
    }
    return 0;
}

static int l_set_pos(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        auto vec = lua::tovec3(L, 2);
        check_valid(vec);
        entity->getTransform().setPos(vec);
        entity->getRigidbody().hitbox.setPos(vec);
    }
    return 0;
}

static int l_get_size(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushvec3(L, entity->getTransform().size);
    }
    return 0;
}

static int l_set_size(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        auto vec = lua::tovec3(L, 2);
        check_valid(vec);
        entity->getTransform().setSize(vec);
    }
    return 0;
}

static int l_get_rot(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        return lua::pushmat4(L, entity->getTransform().rot);
    }
    return 0;
}

static int l_set_rot(lua::State* L) {
    if (auto entity = get_entity(L, 1)) {
        auto matrix = lua::tomat4(L, 2);
        check_valid(matrix);
        entity->getTransform().setRot(matrix);
    }
    return 0;
}

const luaL_Reg transformlib[] = {
    {"get_pos", lua::wrap<l_get_pos>},
    {"set_pos", lua::wrap<l_set_pos>},
    {"get_size", lua::wrap<l_get_size>},
    {"set_size", lua::wrap<l_set_size>},
    {"get_rot", lua::wrap<l_get_rot>},
    {"set_rot", lua::wrap<l_set_rot>},
    {nullptr, nullptr}
};
