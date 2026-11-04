#pragma once

#include "Hitbox.hpp"
#include "SpatialGrid.hpp"

#include "typedefs.hpp"

#include <vector>
#include <glm/glm.hpp>

class BroadPhase {
public:
    BroadPhase(
        const std::vector<Hitbox*>& hitboxes,
        const std::vector<Hitbox*>& solidHitboxes,
        const glm::vec3& gravity
    );

    void build(float time);

    void update(size_t index, float time);

    const std::vector<Hitbox*>& getNearbySolids(size_t index) const {
        return nearbySolids[index];
    }
private:
    const std::vector<Hitbox*>& hitboxes;
    const std::vector<Hitbox*>& solidHitboxes;
    glm::vec3 gravity;
    SpatialGrid grid;
    std::vector<std::vector<Hitbox*>> nearbySolids;
    std::vector<glm::vec3> nearbyOrigins;
    std::vector<float> nearbyBudgets;
    std::vector<glm::vec3> nearbyReaches;
    std::vector<glm::vec3> solidOrigins;
    std::vector<float> solidBudgets;
    std::vector<uint> candidates;
    std::vector<size_t> solidStamps;
    std::vector<uint> movedSolids;
    std::vector<bool> solidMoved;
    std::vector<std::pair<Hitbox*, uint>> solidIndices;
    std::vector<int> hitboxSolids;
    size_t solidStamp = 0;
    float maxSolidHalfY = 0.0f;

    void buildGrid();

    void indexSolids();

    uint getSolidIndex(Hitbox* box) const;

    bool isSolidNearby(size_t index, uint solid) const;

    void collectNearbySolids(size_t index);

    void rebuildNearbySolids(size_t index, float time);
};
