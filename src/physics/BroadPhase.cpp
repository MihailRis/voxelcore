#include "BroadPhase.hpp"
#include "collisions.hpp"

#include <algorithm>

inline constexpr size_t GRID_SCAN_RATIO = 8;

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

static glm::vec3 calc_nearby_reach(
    const Hitbox& hitbox, float maxSolidHalfY, float budget
) {
    auto ext = calc_extents(hitbox);

    glm::vec3 reach(
        0.5f,
        hitbox.stepHeight * 2.0f + E * 8.0f + ext.y * 0.5f +
            maxSolidHalfY + 0.5f + MAX_FIX * 2.0f,
        0.5f
    );
    reach += ext + budget;
    return reach;
}

BroadPhase::BroadPhase(
    const std::vector<Hitbox*>& hitboxes,
    const std::vector<Hitbox*>& solidHitboxes,
    const glm::vec3& gravity
)
    : hitboxes(hitboxes), solidHitboxes(solidHitboxes), gravity(gravity) {
}

void BroadPhase::build(float time) {
    size_t count = hitboxes.size();
    nearbySolids.resize(count);
    nearbyOrigins.resize(count);
    nearbyBudgets.resize(count);
    nearbyReaches.resize(count);
    solidBudgets.resize(solidHitboxes.size());
    solidOrigins.resize(solidHitboxes.size());
    solidStamps.assign(solidHitboxes.size(), 0);
    solidStamp = 0;

    maxSolidHalfY = 0.0f;
    for (size_t j = 0; j < solidHitboxes.size(); j++) {
        const auto& box = *solidHitboxes[j];
        solidBudgets[j] = calc_move_budget(box, gravity, time);
        solidOrigins[j] = box.position;
        maxSolidHalfY = glm::max(maxSolidHalfY, calc_extents(box).y);
    }
    buildGrid();

    for (size_t i = 0; i < count; i++) {
        const auto& hitbox = *hitboxes[i];
        nearbyOrigins[i] = hitbox.position;
        nearbyBudgets[i] = calc_move_budget(hitbox, gravity, time);
        nearbyReaches[i] =
            calc_nearby_reach(hitbox, maxSolidHalfY, nearbyBudgets[i]);
        collectNearbySolids(i);
    }
    indexSolids();
}

void BroadPhase::update(size_t index, float time) {
    auto moved = glm::abs(hitboxes[index]->position - nearbyOrigins[index]);
    if (glm::max(moved.x, glm::max(moved.y, moved.z)) > nearbyBudgets[index]) {
        rebuildNearbySolids(index, time);
    }
}

void BroadPhase::buildGrid() {
    grid.clear();
    movedSolids.clear();
    solidMoved.assign(solidHitboxes.size(), false);

    for (uint j = 0; j < solidHitboxes.size(); j++) {
        grid.insert(
            j,
            solidOrigins[j],
            calc_extents(*solidHitboxes[j]) + solidBudgets[j]
        );
    }
    grid.build();
}

void BroadPhase::indexSolids() {
    solidIndices.resize(solidHitboxes.size());
    for (size_t j = 0; j < solidHitboxes.size(); j++) {
        solidIndices[j] = {solidHitboxes[j], static_cast<uint>(j)};
    }
    std::sort(solidIndices.begin(), solidIndices.end());

    hitboxSolids.resize(hitboxes.size());
    for (size_t i = 0; i < hitboxes.size(); i++) {
        auto it = std::lower_bound(
            solidIndices.begin(),
            solidIndices.end(),
            std::make_pair(hitboxes[i], 0u)
        );
        bool found = it != solidIndices.end() && it->first == hitboxes[i];
        hitboxSolids[i] = found ? static_cast<int>(it->second) : -1;
    }
}

uint BroadPhase::getSolidIndex(Hitbox* box) const {
    return std::lower_bound(
        solidIndices.begin(), solidIndices.end(), std::make_pair(box, 0u)
    )->second;
}

bool BroadPhase::isSolidNearby(size_t index, uint solid) const {
    auto limit = nearbyReaches[index] +
                 calc_extents(*solidHitboxes[solid]) + solidBudgets[solid];
    auto diff = glm::abs(solidOrigins[solid] - nearbyOrigins[index]);
    return diff.x <= limit.x && diff.y <= limit.y && diff.z <= limit.z;
}

void BroadPhase::collectNearbySolids(size_t index) {
    auto& solids = nearbySolids[index];
    solids.clear();
    auto testSolid = [&](uint j) {
        if (isSolidNearby(index, j)) {
            solids.push_back(solidHitboxes[j]);
        }
    };

    size_t stamp = ++solidStamp;
    candidates.clear();
    auto addCandidate = [&](uint j) {
        if (solidStamps[j] != stamp) {
            solidStamps[j] = stamp;
            candidates.push_back(j);
        }
    };
    if (!grid.query(nearbyOrigins[index], nearbyReaches[index], addCandidate)) {
        for (uint j = 0; j < solidHitboxes.size(); j++) {
            testSolid(j);
        }
        return;
    }
    for (uint j : movedSolids) {
        addCandidate(j);
    }
    if (candidates.size() * GRID_SCAN_RATIO > solidHitboxes.size()) {
        for (uint j = 0; j < solidHitboxes.size(); j++) {
            if (solidStamps[j] == stamp) {
                testSolid(j);
            }
        }
        return;
    }
    std::sort(candidates.begin(), candidates.end());
    for (uint j : candidates) {
        testSolid(j);
    }
}

void BroadPhase::rebuildNearbySolids(size_t index, float time) {
    const auto& hitbox = *hitboxes[index];
    nearbyOrigins[index] = hitbox.position;
    nearbyBudgets[index] = calc_move_budget(hitbox, gravity, time);
    nearbyReaches[index] =
        calc_nearby_reach(hitbox, maxSolidHalfY, nearbyBudgets[index]);

    int solid = hitboxSolids[index];
    if (solid >= 0) {
        solidOrigins[solid] = hitbox.position;
        solidBudgets[solid] = nearbyBudgets[index];
        if (!solidMoved[solid]) {
            solidMoved[solid] = true;
            movedSolids.push_back(solid);
        }
        if (movedSolids.size() * GRID_SCAN_RATIO > solidHitboxes.size()) {
            buildGrid();
        }
    }
    collectNearbySolids(index);
    if (solid < 0) {
        return;
    }
    auto box = solidHitboxes[solid];
    for (size_t i = 0; i < hitboxes.size(); i++) {
        if (i == index || !isSolidNearby(i, solid)) {
            continue;
        }
        auto& list = nearbySolids[i];
        auto it = std::lower_bound(
            list.begin(),
            list.end(),
            static_cast<uint>(solid),
            [this](Hitbox* other, uint value) {
                return getSolidIndex(other) < value;
            }
        );
        if (it == list.end() || *it != box) {
            list.insert(it, box);
        }
    }
}
