#pragma once

#include <optional>

#include "frontend/hud.hpp"
#include "objects/Entities.hpp"
#include "objects/Entity.hpp"
#include "world/Level.hpp"
#include "logic/LevelController.hpp"
#include "api_lua.hpp"

namespace scripting {
    extern Hud* hud;
}

inline std::optional<Entity> get_entity(lua::State* L, int idx) {
    auto id = lua::tointeger(L, idx);
    auto level = scripting::controller->getLevel();
    return level->entities->get(id);
}
