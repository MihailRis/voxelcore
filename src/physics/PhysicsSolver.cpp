#include "PhysicsSolver.hpp"
#include "Hitbox.hpp"

#include "debug/Logger.hpp"
#include "maths/aabb.hpp"
#include "voxels/Block.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/voxel.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

const float E = 0.03f;
const float MAX_FIX = 0.1f;

PhysicsSolver::PhysicsSolver(glm::vec3 gravity) : gravity(gravity) {
}

void PhysicsSolver::step(
    const GlobalChunks& chunks, 
    Hitbox& hitbox, 
    float delta, 
    uint substeps, 
    entityid_t entity
) {
    float dt = delta / static_cast<float>(substeps);
    float linearDamping = hitbox.linearDamping;

    const glm::vec3& half = hitbox.halfsize;
    glm::vec3& pos = hitbox.position;
    glm::vec3& vel = hitbox.velocity;
    float gravityScale = hitbox.gravityScale;
    
    bool prevGrounded = hitbox.grounded;
    hitbox.grounded = false;
    for (uint i = 0; i < substeps; i++) {
        float px = pos.x;
        float py = pos.y;
        float pz = pos.z;
        
        vel += gravity * dt * gravityScale;
        if (hitbox.type == BodyType::DYNAMIC) {
            colisionCalc(chunks, hitbox, vel, pos, half, 
                         (prevGrounded && gravityScale > 0.0f) ? 0.5f : 0.0f);
        }
        vel.x *= glm::max(0.0f, 1.0f - dt * linearDamping);
        if (hitbox.verticalDamping) {
            vel.y *= glm::max(0.0f, 1.0f - dt * linearDamping);
        }
        vel.z *= glm::max(0.0f, 1.0f - dt * linearDamping);

        pos += vel * dt + gravity * gravityScale * dt * dt * 0.5f;
        if (hitbox.grounded && pos.y < py) {
            pos.y = py;
        }

        if (hitbox.crouching && hitbox.grounded) {
            const float crouch_limit = -2.0f/BLOCK_AABB_GRID; // offset between edge and crouch box
            AABB aabbx = hitbox.getAABB();
            AABB aabbz = hitbox.getAABB();
            aabbx.increment({crouch_limit+std::abs(px-pos.x)*2, 2*E, crouch_limit});
            aabbz.increment({crouch_limit, 2*E, crouch_limit+std::abs(pz-pos.z)*2});
            if (!chunks.isObstacleWith(aabbx)) {
                pos.z = pz;
            }
            if (!chunks.isObstacleWith(aabbz)) {
                pos.x = px;
            }
        }
    }
    AABB aabb = hitbox.getAABB();
    for (size_t i = 0; i < sensors.size(); i++) {
        auto& sensor = *sensors[i];
        if (sensor.entity == entity) {
            continue;
        }

        bool triggered = false;
        switch (sensor.type) {
            case SensorType::AABB:
                triggered = aabb.intersect(sensor.calculated.aabb);
                break;
            case SensorType::RADIUS:
                triggered = glm::distance2(
                    hitbox.position, glm::vec3(sensor.calculated.radial))
                     < sensor.calculated.radial.w;
                break;
        }
        if (triggered) {
            if (sensor.prevEntered.find(entity) == sensor.prevEntered.end()) {
                sensor.enterCallback(sensor.entity, sensor.index, entity);
            }
            sensor.nextEntered.insert(entity);
        }
    }
}

/// @brief calculate max stepHeight if something above
static float calc_step_height(
    const GlobalChunks& chunks,
    Hitbox& hitbox,
    float stepHeight
) {
    if (stepHeight > 0.0f) {
        AABB step_aabb = hitbox.getAABB();
        step_aabb.translated({0, stepHeight, 0});
        if (chunks.isObstacleWith(step_aabb)) {
            return 0.0f;
        }
    }
    
    return stepHeight;
}


// TODO: concatenate neg and pos functions

/// @brief push out when collided (-x-y-z)
template <int nx, int ny, int nz>
static bool calc_collision_neg(
    const GlobalChunks& chunks,
    glm::vec3& pos,
    glm::vec3& vel,
    const glm::vec3& half
) {
    if (vel[nx] >= 0.0f) {
        return false;
    }

    glm::vec3 coord = pos;
    coord[nx] -= half[nx]+E;
    glm::vec3 size = half-E;
    size[nx] = 0;
    if (const auto aabb = chunks.isObstacleWith(coord.x, coord.y, coord.z, size)) {
        vel[nx] = 0.0f;
        float newx = floor(coord[nx]) + aabb->max()[nx] + half[nx] + E;
        if (std::abs(newx-pos[nx]) <= MAX_FIX) {
            pos[nx] = newx;
        }
        return true;
    }
    return false;
}

/// @brief push out when collided (+x+y+z)
template <int nx, int ny, int nz>
static bool calc_collision_pos(
    const GlobalChunks& chunks,
    glm::vec3& pos,
    glm::vec3& vel,
    const glm::vec3& half
) {
    if (vel[nx] <= 0.0f) {
        return false;
    }

    glm::vec3 coord = pos;
    coord[nx] += half[nx]+E;
    glm::vec3 size = half-E;
    size[nx] = 0;
    if (const auto aabb = chunks.isObstacleWith(coord.x, coord.y, coord.z, size)) {
        vel[nx] = 0.0f;
        float newx = floor(coord[nx]) + aabb->min()[nx] - half[nx] - E;
        if (std::abs(newx-pos[nx]) <= MAX_FIX) {
            pos[nx] = newx;
        }
        return true;
    }
    return false;
}

void PhysicsSolver::colisionCalc(
    const GlobalChunks& chunks,
    Hitbox& hitbox,
    glm::vec3& vel,
    glm::vec3& pos,
    const glm::vec3 half,
    float stepHeight
) {
    stepHeight = calc_step_height(chunks, hitbox, stepHeight);

    const AABB* aabb;
    
    calc_collision_neg<0, 1, 2>(chunks, pos, vel, half);
    calc_collision_pos<0, 1, 2>(chunks, pos, vel, half);

    calc_collision_neg<2, 1, 0>(chunks, pos, vel, half);
    calc_collision_pos<2, 1, 0>(chunks, pos, vel, half);

    if (calc_collision_neg<1, 0, 2>(chunks, pos, vel, half)) {
            hitbox.grounded = true;
    }

    if (stepHeight > 0.0 && vel.y <= 0.0f) {
        if ((aabb = chunks.isObstacleWith(pos.x, pos.y-half.y+E, pos.z, {half.x-E, 0, half.z-E}))) {
            vel.y = 0.0f;
            float newy = floor(pos.y-half.y+E) + aabb->max().y + half.y;
            if (std::abs(newy-pos.y) <= MAX_FIX+stepHeight) {
                pos.y = newy;
            }
        }
    }
    if (vel.y > 0.0f) {
        if ((aabb = chunks.isObstacleWith(pos.x, pos.y+half.y+E, pos.z, {half.x-E, 0, half.z-E}))) {
            vel.y = 0.0f;
            float newy = floor(pos.y+half.y+E) - half.y + aabb->min().y - E;
            if (std::abs(newy-pos.y) <= MAX_FIX) {
                pos.y = newy;
            }
        }
    }
}

bool PhysicsSolver::isBlockInside(int x, int y, int z, Hitbox* hitbox) {
    const glm::vec3& pos = hitbox->position;
    const glm::vec3& half = hitbox->halfsize;
    return x >= floor(pos.x-half.x) && x <= floor(pos.x+half.x) &&
           z >= floor(pos.z-half.z) && z <= floor(pos.z+half.z) &&
           y >= floor(pos.y-half.y) && y <= floor(pos.y+half.y);
}

bool PhysicsSolver::isBlockInside(int x, int y, int z, Block* def, blockstate state, Hitbox* hitbox) {
    const float e = 0.001f; // inaccuracy
    const glm::vec3& pos = hitbox->position;
    const glm::vec3& half = hitbox->halfsize;
    const auto& boxes = def->rotatable 
                      ? def->rt.hitboxes[state.rotation] 
                      : def->hitboxes;
    for (const auto& block_hitbox : boxes) {
        glm::vec3 min = block_hitbox.min();
        glm::vec3 max = block_hitbox.max();
        if (min.x < pos.x+half.x-x-e && max.x > pos.x-half.x-x+e &&
            min.z < pos.z+half.z-z-e && max.z > pos.z-half.z-z+e &&
            min.y < pos.y+half.y-y-e && max.y > pos.y-half.y-y+e)
            return true;
    }
    return false;
}

void PhysicsSolver::removeSensor(Sensor* sensor) {
    sensors.erase(std::remove(sensors.begin(), sensors.end(), sensor), sensors.end());
}
