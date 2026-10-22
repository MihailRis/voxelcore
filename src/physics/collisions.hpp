#pragma once

#include "Hitbox.hpp"

#include "typedefs.hpp"

#include <vector>
#include <glm/glm.hpp>

class GlobalChunks;

namespace physics {
    inline constexpr float E = 0.03f;
    inline constexpr float MAX_FIX = 0.1f;

    struct CollisionContext {
        const GlobalChunks& chunks;
        const std::vector<Hitbox*>& solids;
        glm::vec3 gravity;
        float dt;
        uint wakeSubstep;
    };

    void calc_substep(Hitbox& hitbox, const CollisionContext& ctx);
}
