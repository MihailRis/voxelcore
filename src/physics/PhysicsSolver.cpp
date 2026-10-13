#include "PhysicsSolver.hpp"
#include "Hitbox.hpp"

#include "maths/aabb.hpp"
#include "voxels/Block.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/voxel.hpp"
#include "objects/Entities.hpp"
#include "debug/Logger.hpp"

#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

inline constexpr float E = 0.03f;
inline constexpr float MAX_FIX = 0.1f;
inline constexpr int MAX_NEARBY_REBUILDS = 4;

static debug::Logger logger("physics-solver");

PhysicsSolver::PhysicsSolver(const GlobalChunks& chunks, glm::vec3 gravity)
    : chunks(chunks), gravity(std::move(gravity)) {
}

static glm::vec3 calc_collsion_velocity_result(
    const Hitbox& a, const Hitbox& b
) {
    const auto& vA = a.velocity;
    const auto& vB = b.velocity;
    if (glm::isinf(a.mass)) {
        return vA;
    }
    if (glm::isinf(b.mass)) {
        return vB - a.elasticity * (vA - vB);
    }
    const auto& mA = a.mass;
    const auto& mB = b.mass;
    return (mA * vA + mB * vB + a.elasticity * mB * (vB - vA)) / (mA + mB);
}

static float calc_step_height(
    const GlobalChunks& chunks, 
    const glm::vec3& pos, 
    const glm::vec3& half,
    float stepHeight
) {
    AABB aabb(-half, +half);
    aabb.scale(glm::vec3(1.0f - E * 2, 1.0f, 1.0f - E * 2));
    aabb = aabb + pos + glm::vec3(0.0f, stepHeight, 0.0f);
    if (stepHeight <= 0.0f) {
        return stepHeight;
    }
    for (int ix = 0; ix <= glm::ceil((half.x - E) * 2); ix++) {
        float x = (pos.x - half.x) + ix;
        for (int iz = 0; iz <= glm::ceil((half.z - E) * 2); iz++) {
            float z = (pos.z - half.z) + iz;
            if (chunks.isObstacleAt(x, pos.y + half.y + stepHeight, z, aabb)) {
                return 0.0f;
            }
        }
    }
    return stepHeight;
}

// todo: reduce code duplication

template <int nx, int ny, int nz, int sign>
static void calc_collision(
    Hitbox& hitbox,
    const GlobalChunks& chunks,
    const std::vector<Hitbox*>& solidHitboxes,
    const glm::vec3& half,
    float stepHeight
) {
    auto& pos = hitbox.position;
    auto& vel = hitbox.velocity;

    glm::vec3 offset(0.0f, stepHeight + E, 0.0f);
    for (auto box : solidHitboxes) {
        if (glm::distance2(box->position, pos) < E) {
            continue;
        }
        auto boxhalf = box->getHalfSize();
        auto aabb = AABB(pos - half, pos + half);
        glm::vec3 scale(1.0f);
        scale[nz] = 1.0f - E * 8.0f;
        scale[ny] = 1.0f - E * 2.0f;
        aabb.scale(scale);
        aabb = aabb + offset;
        aabb.b.y -= stepHeight + E * 2;

        if ((box->position[nx] - pos[nx]) * sign <= 0.0f || !box->getAABB().intersects(aabb)) {
            continue;
        }

        float newnegx = box->position[nx] - boxhalf[nx] - half[nx];
        float newposx = box->position[nx] + boxhalf[nx] + half[nx];
        float newx;
        if ((glm::abs(newnegx - pos[nx]) - glm::abs(newposx - pos[nx]))*sign < 0.0f) {
            newx = sign > 0 ? newnegx : newposx;
        } else {
            continue;
        }
        if ((pos[nx] - newx) * sign > 0.0f && glm::abs(pos[nx] - newx) < MAX_FIX) {
            auto velA = calc_collsion_velocity_result(hitbox, *box);
            auto velB = calc_collsion_velocity_result(*box, hitbox);
            if ((vel[nx] - box->velocity[nx]) * sign > 0.0f) {
                vel[nx] = velA[nx];
                box->velocity[nx] = velB[nx];
            }
            pos[nx] = newx;
        }
    }
    if (vel[nx] * sign <= 0.0f && hitbox.groundVelocity[nx] * sign <= 0.0f) {
        return;
    }
    for (int iy = 0; iy <= glm::ceil(((half - offset * 0.5f)[ny] - E) * 2); iy++) {
        glm::vec3 coord;
        coord[ny] = ((pos + offset)[ny] - half[ny] + E) + iy;
        for (int iz = 0; iz <= glm::ceil((half[nz] - E) * 2); iz++) {
            coord[nz] = (pos[nz] - half[nz] + E) + iz;
            coord[nx] = pos[nx] + (half[nx] + E * 2) * sign;

            auto aabb = AABB(pos - half, pos + half);
            glm::vec3 scale(1.0f);
            scale[nz] = 1.0f - E * 2.0f;
            aabb.scale(scale);
            aabb = aabb + offset;
            aabb.b.y -= stepHeight + E * 2;

            if (const auto obstacle = chunks.isObstacleAt(coord.x, coord.y, coord.z, aabb)) {
                float newx = std::floor(coord[nx]) - half[nx] * sign +
                             (sign > 0 ? obstacle->min() : obstacle->max())[nx];
                if ((pos[nx] - newx) * sign > 0.0f && glm::abs(pos[nx] - newx) < MAX_FIX) {
                    vel[nx] = -hitbox.elasticity * vel[nx];
                    pos[nx] = newx;
                }
                return;
            }
        }
    }
}

bool PhysicsSolver::calcCollisionNegY(
    Hitbox& hitbox,
    const glm::vec3& half,
    float dt,
    const std::vector<Hitbox*>& solids
) {
    auto& pos = hitbox.position;
    auto& vel = hitbox.velocity;

    for (auto box : solids) {
        if (glm::distance2(box->position, pos) < E) {
            continue;
        }
        auto aabb = AABB(pos - half, pos + half);
        glm::vec3 scale(1.0f);
        scale.x = 1.0f - E * 4.0f;
        scale.z = 1.0f - E * 4.0f;
        aabb.scale(scale);

        auto boxhalf = box->getHalfSize();
        if (box->position.y < pos.y && box->getAABB().intersects(aabb)) {
            float newy = box->position.y + boxhalf.y + half.y;
            if (pos.y < newy && glm::abs(pos.y - newy) < boxhalf.y) {
                pos.y = newy;
            }

            auto velA = calc_collsion_velocity_result(hitbox, *box);
            auto velB = calc_collsion_velocity_result(*box, hitbox);

            hitbox.groundVelocity = (box->position - box->prevPosition) / dt;
            if (vel.y < hitbox.groundVelocity.y) {
                vel.y = velA.y;
                box->velocity.y = velB.y;
                if (hitbox.groundMaterial.empty() && !box->material.empty()) {
                    hitbox.groundMaterial = box->material;
                }
                return true;
            }
        }
    }
    if (vel.y >= 0.0f) {
        return false;
    }
    for (int ix = 0; ix <= glm::ceil((half.x - E) * 2); ix++) {
        glm::vec3 coord;
        coord.x = (pos.x - half.x + E) + ix;
        for (int iz = 0; iz <= glm::ceil((half.z - E) * 2); iz++) {
            coord.z = (pos.z - half.z + E) + iz;
            coord.y = pos.y - half.y - E * 2;

            auto aabb = AABB(pos - half, pos + half);
            glm::vec3 scale(1.0f);
            scale.x = 1.0f - E * 4.0f;
            scale.z = 1.0f - E * 4.0f;
            aabb.scale(scale);

            if (const auto obstacle =
                    chunks.isObstacleAt(coord.x, coord.y, coord.z, aabb)) {
                float newy = std::floor(coord.y) + half.y + obstacle->max().y;

                if (newy >= pos.y) {
                    vel.y = -hitbox.elasticity * vel.y;
                    pos.y = newy;
                }
                hitbox.groundVelocity = {};
                return true;
            }
        }
    }
    return false;
}

void PhysicsSolver::calcCollisions(
    Hitbox& hitbox,
    glm::vec3& vel,
    glm::vec3& pos,
    const glm::vec3& half,
    float stepHeight,
    float dt,
    const std::vector<Hitbox*>& solids
) {
    stepHeight = calc_step_height(chunks, pos, half, stepHeight);

    auto prevPos = pos;

    calc_collision<0, 1, 2, -1>(hitbox, chunks, solids, half, stepHeight);
    calc_collision<0, 1, 2, 1>(hitbox, chunks, solids, half, stepHeight);

    float xpos = pos.x;
    pos.x = prevPos.x;

    calc_collision<2, 1, 0, -1>(hitbox, chunks, solids, half, stepHeight);
    calc_collision<2, 1, 0, 1>(hitbox, chunks, solids, half, stepHeight);
    pos.x = xpos;

    if (calcCollisionNegY(hitbox, half, dt, solids)) {
        hitbox.grounded = true;
    }

    if (vel.y > 0.0f){
        AABB boxAABB = AABB(-half, +half);
        boxAABB.scale(glm::vec3(1.0f - E * 4, 1.0f + E * 2, 1.0f - E * 4));
        boxAABB = boxAABB.translated(pos);

        for (int ix = 0; ix <= glm::ceil((half.x - E) * 2); ix++) {
            float x = (pos.x - half.x + E) + ix;
            for (int iz = 0; iz <= glm::ceil((half.z - E) * 2); iz++) {
                float z = (pos.z - half.z + E) + iz;
                float y = (pos.y + half.y + E) - E;
                if (auto aabb = chunks.isObstacleAt(x, y, z, boxAABB)) {
                    float newy = std::floor(y) - half.y + aabb->min().y - E;
                    if (pos.y >= newy) {
                        vel.y = -hitbox.elasticity * vel.y;
                        pos.y = newy;
                    }
                    break;
                }
            }
        }
        for (auto box : solids) {
            if (glm::distance2(box->position, pos) < E) {
                continue;
            }
            auto boxhalf = box->getHalfSize();
            if (box->position.y > pos.y && box->getAABB().intersects(boxAABB)) {
                float newy = box->position.y - boxhalf.y - half.y;
                if (pos.y > newy && glm::abs(pos.y - newy) < 0.5f) {
                    pos.y = newy;
                }
                auto velA = calc_collsion_velocity_result(hitbox, *box);
                auto velB = calc_collsion_velocity_result(*box, hitbox);
                if (vel.y > hitbox.groundVelocity.y) {
                    vel.y = velA.y;
                    box->velocity.y = velB.y;
                    break;
                }
            }
        }
    }

    // step on
    if (stepHeight > 0.0 && vel.y <= 0.0f) {
        AABB boxAABB = AABB(pos - half, pos + half);
        boxAABB.scale(glm::vec3(1.0f - E * 4.0f, 1.0f - E * 2, 1.0f - E * 4.0f));
        for (int ix = 0; ix <= glm::ceil((half.x - E) * 2); ix++) {
            float x = (pos.x - half.x) + ix;
            for (int iz = 0; iz <= glm::ceil((half.z - E) * 2); iz++) {
                float z = (pos.z - half.z) + iz;
                float y = (pos.y - half.y + E) + E;
                if (auto aabb = chunks.isObstacleAt(x, y, z, boxAABB)) {
                    if (vel.y < 0.0f) {
                        vel.y = 0.0f;
                    }
                    float newy = std::floor(y) + aabb->max().y + half.y;
                    if (std::abs(newy - pos.y) <= stepHeight) {
                        pos.y = newy;
                    }
                    break;
                }
            }
        }
        for (auto box : solids) {
            if (glm::distance2(box->position, pos) < E) {
                continue;
            }
            auto boxhalf = box->getHalfSize();
            if (box->getAABB().intersects(boxAABB)) {
                vel.y = 0.0f;
                float newy = box->position.y + boxhalf.y + half.y;
                if (std::abs(newy - pos.y) <= stepHeight + E * 4) {
                    pos.y = newy;
                }
            }
        }
    }
}

void PhysicsSolver::calcSubstep(
    Hitbox& hitbox,
    glm::vec3& vel,
    glm::vec3& pos,
    float dt,
    const std::vector<Hitbox*>& solids
) {
    auto initpos = pos;
    auto half = hitbox.getHalfSize();
    float gravityScale = hitbox.gravityScale;
        
    if (hitbox.type == BodyType::DYNAMIC) {
        calcCollisions(
            hitbox,
            vel,
            pos,
            half,
            (hitbox.prevGrounded && gravityScale > 0.0f) ? hitbox.stepHeight
                                                         : 0.0f,
            dt,
            solids
        );
    }

    vel += gravity * dt * gravityScale;
    pos += vel * dt + gravity * gravityScale * dt * dt * 0.5f;

    // crouching
    if (!hitbox.crouching || !hitbox.grounded) {
        return;
    }
    float y = (pos.y - half.y - E);
    for (int axis = 0; axis <= 2; axis += 2) {
        hitbox.grounded = false;

        auto checkPos = pos;
        checkPos.x = axis != 0 ? initpos.x : pos.x;
        checkPos.z = axis == 0 ? initpos.z : pos.z;

        AABB boxAABB(checkPos - half, checkPos + half);
        boxAABB.scale(glm::vec3(1.0f - E * 4, 1.5f, 1.0f - E * 4));
        for (int ix = 0; ix <= glm::ceil((half.x - E) * 2); ix++) {
            float x = (pos.x - half.x + E) + ix;
            for (int iz = 0; iz <= glm::ceil((half.z - E) * 2); iz++){
                float z = (pos.z - half.z + E) + iz;
                if (chunks.isObstacleAt(x, y, z, boxAABB)){
                    hitbox.grounded = true;
                    break;
                }
            }
        }
        for (auto box : solids) {
            if (glm::distance2(box->position, pos) < E) {
                continue;
            }
            if (box->position.y < pos.y && box->getAABB().intersects(boxAABB)) {
                hitbox.grounded = true;
                break;
            }
        }
        if (!hitbox.grounded) {
            pos[axis] = initpos[axis];
            vel[axis] = 0.0f;
        }
    }
    hitbox.grounded = true;
}

static float calc_move_budget(
    const Hitbox& hitbox, const glm::vec3& gravity, float time
) {
    float speed = glm::length(hitbox.velocity) * 1.25f +
                  glm::length(gravity) * glm::abs(hitbox.gravityScale) * time;
    return speed * time + 0.05f;
}

static glm::vec3 calc_extents(const Hitbox& hitbox) {
    return glm::max(hitbox.halfsize, hitbox.getHalfSize());
}

void PhysicsSolver::findNearbySolids(float time) {
    size_t count = hitboxes.size();
    nearbySolids.resize(count);
    nearbyOrigins.resize(count);
    nearbyBudgets.resize(count);
    solidBudgets.resize(solidHitboxes.size());

    float maxSolidHalfY = 0.0f;
    for (size_t j = 0; j < solidHitboxes.size(); j++) {
        const auto& box = *solidHitboxes[j];
        solidBudgets[j] = calc_move_budget(box, gravity, time);
        maxSolidHalfY = glm::max(maxSolidHalfY, calc_extents(box).y);
    }
    for (size_t i = 0; i < count; i++) {
        const auto& hitbox = *hitboxes[i];
        auto& solids = nearbySolids[i];
        solids.clear();
        nearbyOrigins[i] = hitbox.position;
        nearbyBudgets[i] = calc_move_budget(hitbox, gravity, time);

        auto ext = calc_extents(hitbox);

        glm::vec3 reach(
            0.5f,
            hitbox.stepHeight * 2.0f + E * 8.0f + ext.y * 0.5f +
                maxSolidHalfY + 0.5f + MAX_FIX * 2.0f,
            0.5f
        );
        reach += ext + nearbyBudgets[i];
        for (size_t j = 0; j < solidHitboxes.size(); j++) {
            const auto& box = *solidHitboxes[j];
            auto limit = reach + calc_extents(box) + solidBudgets[j];
            auto diff = glm::abs(box.position - hitbox.position);
            if (diff.x <= limit.x && diff.y <= limit.y && diff.z <= limit.z) {
                solids.push_back(solidHitboxes[j]);
            }
        }
    }
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
    findNearbySolids(delta);

    int rebuildsLeft = MAX_NEARBY_REBUILDS;
    for (uint i = 0; i < substeps; i++) {
        for (size_t j = 0; j < hitboxes.size(); j++) {
            auto hitbox = hitboxes[j];
            glm::vec3& pos = hitbox->position;
            hitbox->prevPosition = hitbox->position;
            calcSubstep(
                *hitbox,
                hitbox->velocity,
                pos,
                dt,
                rebuildsLeft >= 0 ? nearbySolids[j] : solidHitboxes
            );
            if (rebuildsLeft < 0) {
                continue;
            }

            auto moved = glm::abs(pos - nearbyOrigins[j]);
            if (glm::max(moved.x, glm::max(moved.y, moved.z)) >
                nearbyBudgets[j]) {
                if (--rebuildsLeft >= 0) {
                    findNearbySolids(dt * static_cast<float>(substeps - i));
                }
            }
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
        
        updateSensors(*hitbox);
    }
}

void PhysicsSolver::updateSensors(Hitbox& hitbox) {
    auto aabb = hitbox.getAABB();

    for (size_t i = 0; i < sensors.size(); i++) {
        auto& sensor = *sensors[i];
        if (sensor.entity == hitbox.entity) {
            continue;
        }
        bool triggered = false;
        switch (sensor.type) {
            case SensorType::AABB:
                triggered = aabb.intersects(sensor.calculated.aabb);
                break;
            case SensorType::RADIUS:
                triggered = glm::distance2(
                    hitbox.position, glm::vec3(sensor.calculated.radial))
                    < sensor.calculated.radial.w;
                break;
        }
        if (!triggered) {
            continue;
        }
        if (sensor.prevEntered.find(hitbox.entity) == sensor.prevEntered.end()) {
            sensor.enterCallback(sensor.entity, sensor.index, hitbox.entity);
        }
        sensor.nextEntered.insert(hitbox.entity);
    }
}

void PhysicsSolver::removeSensor(Sensor* sensor) {
    sensors.erase(
        std::remove(sensors.begin(), sensors.end(), sensor), sensors.end()
    );
}
