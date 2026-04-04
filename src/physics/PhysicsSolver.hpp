#pragma once

#include "Hitbox.hpp"

#include "typedefs.hpp"
#include "voxels/voxel.hpp"

#include <vector>
#include <glm/glm.hpp>

class Block;
class Entities;
class GlobalChunks;
struct Sensor;

class PhysicsSolver {
public:
    PhysicsSolver(const GlobalChunks& chunks, glm::vec3 gravity);

    void step(const GlobalChunks& chunks, float delta, uint substeps);

    auto& getSensorsWriteable() {
        return sensors;
    }

    auto& getSolidHitboxesWriteable() {
        return solidHitboxes;
    }

    auto& getHitboxesWriteable() {
        return hitboxes;
    }

    void removeSensor(Sensor* sensor);
private:
    const GlobalChunks& chunks;
    glm::vec3 gravity;
    std::vector<Sensor*> sensors;
    std::vector<Hitbox*> solidHitboxes;
    std::vector<Hitbox*> hitboxes;

    void calcCollisions(
        Hitbox& hitbox,
        glm::vec3& vel,
        glm::vec3& pos,
        const glm::vec3& half,
        float stepHeight,
        float dt
    );

    void calcSubstep(Hitbox& hitbox, glm::vec3& vel, glm::vec3& pos, float dt);

    bool calcCollisionNegY(Hitbox& hitbox, const glm::vec3& half, float dt);

    void updateSensors(Hitbox& hitbox);
};
