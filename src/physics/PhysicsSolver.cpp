#include "PhysicsSolver.hpp"
#include "Hitbox.hpp"

#include "maths/aabb.hpp"
#include "voxels/Block.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/voxel.hpp"

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
    float linearDamping = hitbox.linearDamping * hitbox.friction;
    float s = 2.0f/BLOCK_AABB_GRID;

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

        pos += vel * dt + gravity * gravityScale * dt * dt * 0.5f;
        if (hitbox.grounded && pos.y < py) {
            pos.y = py;
        }

        if (hitbox.crouching && hitbox.grounded){
            float y = (pos.y-half.y-E);
            hitbox.grounded = false;
            for (int ix = 0; ix <= (half.x-E)*2/s; ix++) {
                float x = (px-half.x+E) + ix * s;
                for (int iz = 0; iz <= (half.z-E)*2/s; iz++){
                    float z = (pos.z-half.z+E) + iz * s;
                    if (chunks.isObstacleAt(x,y,z)){
                        hitbox.grounded = true;
                        break;
                    }
                }
            }
            if (!hitbox.grounded) {
                pos.z = pz;
            }
            hitbox.grounded = false;
            for (int ix = 0; ix <= (half.x-E)*2/s; ix++) {
                float x = (pos.x-half.x+E) + ix * s;
                for (int iz = 0; iz <= (half.z-E)*2/s; iz++){
                    float z = (pz-half.z+E) + iz * s;
                    if (chunks.isObstacleAt(x,y,z)){
                        hitbox.grounded = true;
                        break;
                    }
                }
            }
            if (!hitbox.grounded) {
                pos.x = px;
            }
            hitbox.grounded = true;
        }
    }
    vel.x /= 1.0f + delta * linearDamping;
    vel.z /= 1.0f + delta * linearDamping;
    if (hitbox.verticalDamping > 0.0f) {
        vel.y /= 1.0f + delta * linearDamping * hitbox.verticalDamping;
    }

    AABB aabb;
    aabb.a = hitbox.position - hitbox.halfsize;
    aabb.b = hitbox.position + hitbox.halfsize;
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

static float calc_step_height(
    const GlobalChunks& chunks, 
    const glm::vec3& pos, 
    const glm::vec3& half,
    float stepHeight,
    float s
) {
    if (stepHeight > 0.0f) {
        for (int ix = 0; ix <= (half.x-E)*2/s; ix++) {
            float x = (pos.x-half.x+E) + ix * s;
            for (int iz = 0; iz <= (half.z-E)*2/s; iz++) {
                float z = (pos.z-half.z+E) + iz * s;
                if (chunks.isObstacleAt(x, pos.y+half.y+stepHeight, z)) {
                    return 0.0f;
                }
            }
        }
    }
    return stepHeight;
}

template <int nx, int ny, int nz>
static bool calc_collision_neg(
    const GlobalChunks& chunks,
    glm::vec3& pos,
    glm::vec3& vel,
    const glm::vec3& half,
    float stepHeight,
    float s
) {
    if (vel[nx] >= 0.0f) {
        return false;
    }
    glm::vec3 offset(0.0f, stepHeight, 0.0f);
    for (int iy = 0; iy <= ((half-offset*0.5f)[ny]-E)*2/s; iy++) {
        glm::vec3 coord;
        coord[ny] = ((pos+offset)[ny]-half[ny]+E) + iy * s;
        for (int iz = 0; iz <= (half[nz]-E)*2/s; iz++){
            coord[nz] = (pos[nz]-half[nz]+E) + iz * s;
            coord[nx] = (pos[nx]-half[nx]-E);

            if (const auto aabb = chunks.isObstacleAt(coord.x, coord.y, coord.z)) {
                vel[nx] = 0.0f;
                float newx = std::floor(coord[nx]) + aabb->max()[nx] + half[nx] + E;
                if (std::abs(newx-pos[nx]) <= MAX_FIX) {
                    pos[nx] = newx;
                }
                return true;
            }
        }
    }
    return false;
}

template <int nx, int ny, int nz>
static void calc_collision_pos(
    const GlobalChunks& chunks,
    glm::vec3& pos,
    glm::vec3& vel,
    const glm::vec3& half,
    float stepHeight,
    float s
) {
    if (vel[nx] <= 0.0f) {
        return;
    }
    glm::vec3 offset(0.0f, stepHeight, 0.0f);
    for (int iy = 0; iy <= ((half-offset*0.5f)[ny]-E)*2/s; iy++) {
        glm::vec3 coord;
        coord[ny] = ((pos+offset)[ny]-half[ny]+E) + iy * s;
        for (int iz = 0; iz <= (half[nz]-E)*2/s; iz++) {
            coord[nz] = (pos[nz]-half[nz]+E) + iz * s;
            coord[nx] = (pos[nx]+half[nx]+E);
            if (const auto aabb = chunks.isObstacleAt(coord.x, coord.y, coord.z)) {
                vel[nx] = 0.0f;
                float newx = std::floor(coord[nx]) - half[nx] + aabb->min()[nx] - E;
                if (std::abs(newx-pos[nx]) <= MAX_FIX) {
                    pos[nx] = newx;
                }
                return;
            }
        }
    }
}

void PhysicsSolver::colisionCalc(
    const GlobalChunks& chunks, 
    Hitbox& hitbox, 
    glm::vec3& vel, 
    glm::vec3& pos, 
    const glm::vec3 half,
    float stepHeight
) {
    // step size (smaller - more accurate, but slower)
    float s = 2.0f/BLOCK_AABB_GRID;

    stepHeight = calc_step_height(chunks, pos, half, stepHeight, s);

    const AABB* aabb;
    
    calc_collision_neg<0, 1, 2>(chunks, pos, vel, half, stepHeight, s);
    calc_collision_pos<0, 1, 2>(chunks, pos, vel, half, stepHeight, s);

    calc_collision_neg<2, 1, 0>(chunks, pos, vel, half, stepHeight, s);
    calc_collision_pos<2, 1, 0>(chunks, pos, vel, half, stepHeight, s);

    if (calc_collision_neg<1, 0, 2>(chunks, pos, vel, half, stepHeight, s)) {
        hitbox.grounded = true;
    }

    if (stepHeight > 0.0 && vel.y <= 0.0f){
        for (int ix = 0; ix <= (half.x-E)*2/s; ix++) {
            float x = (pos.x-half.x+E) + ix * s;
            for (int iz = 0; iz <= (half.z-E)*2/s; iz++) {
                float z = (pos.z-half.z+E) + iz * s;
                float y = (pos.y-half.y+E);
                if ((aabb = chunks.isObstacleAt(x,y,z))){
                    vel.y = 0.0f;
                    float newy = std::floor(y) + aabb->max().y + half.y;
                    if (std::abs(newy-pos.y) <= MAX_FIX+stepHeight) {
                        pos.y = newy;    
                    }
                    break;
                }
            }
        }
    }
    if (vel.y > 0.0f){
        for (int ix = 0; ix <= (half.x-E)*2/s; ix++) {
            float x = (pos.x-half.x+E) + ix * s;
            for (int iz = 0; iz <= (half.z-E)*2/s; iz++) {
                float z = (pos.z-half.z+E) + iz * s;
                float y = (pos.y+half.y+E);
                if ((aabb = chunks.isObstacleAt(x,y,z))){
                    vel.y = 0.0f;
                    float newy = std::floor(y) - half.y + aabb->min().y - E;
                    if (std::abs(newy-pos.y) <= MAX_FIX) {
                        pos.y = newy;
                    }
                    break;
                }
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
