#include "PhysicsSolver.hpp"
#include "Hitbox.hpp"
#include "collisions.hpp"
#include "scheduling.hpp"
#include "sensors.hpp"

#include "voxels/GlobalChunks.hpp"
#include "debug/Logger.hpp"

#include <algorithm>

static debug::Logger logger("physics-solver");

PhysicsSolver::PhysicsSolver(const GlobalChunks& chunks, glm::vec3 gravity)
    : chunks(chunks),
      gravity(gravity),
      broadPhase(hitboxes, solidHitboxes, gravity) {
}

void PhysicsSolver::step(
    const GlobalChunks& chunks, float delta, uint substeps
) {
    for (auto hitbox : hitboxes) {
        hitbox->groundMaterial.clear();
        hitbox->prevGrounded = hitbox->grounded;
        hitbox->grounded = false;
        hitbox->prevVelocity = hitbox->velocity;
    }

    float dt = delta / static_cast<float>(substeps);
    broadPhase.build(delta);
    for (auto hitbox : hitboxes) {
        physics::schedule_substep(*hitbox, gravity, 0, substeps, dt);
    }

    for (uint i = 0; i < substeps; i++) {
        for (size_t j = 0; j < hitboxes.size(); j++) {
            auto hitbox = hitboxes[j];
            if (hitbox->nextSubstep > i) {
                continue;
            }
            hitbox->prevPosition = hitbox->position;
            hitbox->substepTime =
                dt * static_cast<float>(i + 1 - hitbox->doneSubsteps);
            while (hitbox->doneSubsteps <= i) {
                uint left = i + 1 - hitbox->doneSubsteps;
                uint skip =
                    physics::calc_substep_stride(*hitbox, gravity, dt, left) -
                    1;
                if (skip > 0) {
                    physics::integrate(*hitbox, gravity, dt, skip);
                    hitbox->doneSubsteps += skip;
                }
                physics::calc_substep(
                    *hitbox,
                    {this->chunks,
                     broadPhase.getNearbySolids(j),
                     gravity,
                     dt,
                     i + 1}
                );
                hitbox->doneSubsteps++;

                broadPhase.update(j, dt * static_cast<float>(substeps - i));
            }
            physics::schedule_substep(*hitbox, gravity, i + 1, substeps, dt);
        }
    }

    for (auto hitbox : hitboxes) {
        float linearDamping = hitbox->linearDamping;

        if (hitbox->grounded) {
            linearDamping = 10.0f; // TODO: add friction to material
        }

        glm::vec3& vel = hitbox->velocity;
        auto diff = hitbox->groundVelocity - vel;
        vel.x += diff.x * delta * linearDamping;
        vel.z += diff.z * delta * linearDamping;

        if (hitbox->verticalDamping > 0.0f) {
            vel.y /= 1.0f + delta * linearDamping * hitbox->verticalDamping;
        }
        if (!hitbox->grounded) {
            hitbox->groundVelocity *= 1.0f - delta;
        }

        physics::update_sensors(*hitbox, sensors);
    }
}

void PhysicsSolver::removeSensor(Sensor* sensor) {
    sensors.erase(
        std::remove(sensors.begin(), sensors.end(), sensor), sensors.end()
    );
}
