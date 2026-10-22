#include "scheduling.hpp"

inline constexpr float MAX_SUBSTEP_MOVE = 0.03f;
inline constexpr float MAX_SUBSTEP_SINK = 0.015f;

uint physics::calc_substep_stride(
    const Hitbox& hitbox, const glm::vec3& gravity, float dt, uint left
) {
    if (hitbox.crouching) {
        return 1;
    }
    float speed = glm::length(hitbox.velocity);
    float accel = glm::length(gravity) * glm::abs(hitbox.gravityScale) * 1.5f;
    float time;
    if (accel > 0.0f) {
        time = (glm::sqrt(speed * speed + 4.0f * accel * MAX_SUBSTEP_MOVE) -
                speed) / (2.0f * accel);
        time = glm::min(time, glm::sqrt(MAX_SUBSTEP_SINK / accel));
    } else if (speed > 0.0f) {
        time = MAX_SUBSTEP_MOVE / speed;
    } else {
        return left;
    }
    float stride = glm::floor(time / dt);
    if (stride >= static_cast<float>(left)) {
        return left;
    }
    return glm::max(1u, static_cast<uint>(stride));
}

void physics::schedule_substep(
    Hitbox& hitbox, const glm::vec3& gravity, uint done, uint substeps, float dt
) {
    hitbox.doneSubsteps = done;
    if (done >= substeps) {
        hitbox.nextSubstep = substeps;
        return;
    }
    uint left = substeps - done;
    if (left > 1) {
        left--;
    }
    hitbox.nextSubstep =
        done + calc_substep_stride(hitbox, gravity, dt, left) - 1;
}
