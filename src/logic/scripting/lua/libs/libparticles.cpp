#include "libhud.hpp"

#include "graphics/render/ParticlesRenderer.hpp"
#include "graphics/render/Emitter.hpp"
#include "assets/assets_util.hpp"

using namespace scripting;

static int l_emit(lua::State* L) {
    EmitterOrigin origin;
    if (lua::istable(L, 1)) {
        origin = lua::tovec3(L, 1);
    } else {
        origin = static_cast<entityid_t>(lua::tointeger(L, 1));
    }
    int count = lua::tointeger(L, 2);
    auto preset = lua::tovalue(L, 3);
    auto extension = lua::tovalue(L, 4);
    
    ParticlesPreset particlesPreset {};
    particlesPreset.deserialize(preset);
    if (extension != nullptr) {
        particlesPreset.deserialize(extension);
    }
    auto& assets = *engine->getAssets();
    auto region = util::get_texture_region(assets, particlesPreset.texture, "");
    auto emitter = std::make_unique<Emitter>(
        *level,
        std::move(origin),
        std::move(particlesPreset),
        region.texture,
        region.region,
        count
    );
    return lua::pushinteger(L, renderer->particles->add(std::move(emitter)));
}

static int l_stop(lua::State* L) {
    u64id_t id = lua::touinteger(L, 1);
    if (auto emitter = renderer->particles->getEmitter(id)) {
        emitter->stop();
    }
    return 0;
}

static int l_get_origin(lua::State* L) {
    u64id_t id = lua::touinteger(L, 1);
    if (auto emitter = renderer->particles->getEmitter(id)) {
        const auto& origin = emitter->getOrigin();
        if (auto pos = std::get_if<glm::vec3>(&origin)) {
            return lua::pushvec3(L, *pos);
        } else if (auto entityid = std::get_if<entityid_t>(&origin)) {
            return lua::pushinteger(L, *entityid);
        }
    }
    return 0;
}

static int l_set_origin(lua::State* L) {
    u64id_t id = lua::touinteger(L, 1);
    if (auto emitter = renderer->particles->getEmitter(id)) {
        if (lua::istable(L, 2)) {
            emitter->setOrigin(lua::tovec3(L, 2));
        } else {
            emitter->setOrigin(static_cast<entityid_t>(lua::tointeger(L, 2)));
        }
    }
    return 0;
}

static int l_is_alive(lua::State* L) {
    u64id_t id = lua::touinteger(L, 1);
    if (auto emitter = renderer->particles->getEmitter(id)) {
        return lua::pushboolean(L, !emitter->isDead());
    }
    return lua::pushboolean(L, false);
}

const luaL_Reg particleslib[] = {
    {"emit", wrap_hud<l_emit>},
    {"stop", wrap_hud<l_stop>},
    {"is_alive", wrap_hud<l_is_alive>},
    {"get_origin", wrap_hud<l_get_origin>},
    {"set_origin", wrap_hud<l_set_origin>},
    {nullptr, nullptr}
};
