#pragma once

#include "Hitbox.hpp"

#include "typedefs.hpp"

#include <algorithm>
#include <glm/glm.hpp>

inline void integrate(
    Hitbox& box, const glm::vec3& gravity, float dt, uint substeps
) {
    float n = static_cast<float>(substeps);
    auto accel = gravity * box.gravityScale;
    box.position += box.velocity * (dt * n) +
                    accel * (dt * dt * n * (n + 2.0f) * 0.5f);
    box.velocity += accel * (dt * n);
}

inline void push_body(Hitbox& box, int axis, float value, uint wakeSubstep) {
    if (box.velocity[axis] != value) {
        box.velocity[axis] = value;
        box.nextSubstep = std::min(box.nextSubstep, wakeSubstep);
    }
}

uint calc_substep_stride(
    const Hitbox& hitbox, const glm::vec3& gravity, float dt, uint left
);

void schedule_substep(
    Hitbox& hitbox, const glm::vec3& gravity, uint done, uint substeps, float dt
);
