#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "physics/Hitbox.hpp"
#include "Transform.hpp"
#include "Rigidbody.hpp"
#include "ScriptComponents.hpp"
#include "typedefs.hpp"
#include "util/Clock.hpp"

#include <entt/entity/registry.hpp>
#include <unordered_map>

struct EntityDef;

class Level;
class Assets;
class Entity;
class LineBatch;
class ModelBatch;
class Frustum;
class Entities;
class DrawContext;

namespace rigging {
    struct Skeleton;
    class SkeletonConfig;
}

class Entities {
    entt::registry registry;
    Level& level;
    std::unordered_map<entityid_t, entt::entity> entities;
    std::unordered_map<entt::entity, entityid_t> uids;
    entityid_t nextID = 1;
    util::Clock sensorsTickClock;
    util::Clock updateTickClock;

    void updateSensors(
        Rigidbody& body, const Transform& tsf, std::vector<Sensor*>& sensors
    );
    void preparePhysics(float delta);
public:
    struct RaycastResult {
        entityid_t entity;
        glm::ivec3 normal;
        float distance;
    };

    Entities(Level& level);

    void clean();
    void updatePhysics(float delta);
    void update(float delta);

    void renderDebug(
        LineBatch& batch, const Frustum* frustum, const DrawContext& ctx
    );
    void render(
        const Assets& assets,
        ModelBatch& batch,
        const Frustum* frustum,
        float delta,
        bool pause,
        entityid_t fpsEntity
    );

    entityid_t spawn(
        const EntityDef& def,
        glm::vec3 position,
        dv::value args = nullptr,
        dv::value saved = nullptr,
        entityid_t uid = 0
    );

    std::optional<Entity> get(entityid_t id);

    /// @brief Entities raycast. No blocks check included, use combined with
    /// Chunks.rayCast
    /// @param start Ray start
    /// @param dir Ray direction normalized vector
    /// @param maxDistance Max ray length
    /// @param ignore Ignored entity ID
    /// @return An optional structure containing entity, normal and distance
    std::optional<RaycastResult> rayCast(
        glm::vec3 start,
        glm::vec3 dir,
        float maxDistance,
        entityid_t ignore = -1
    );

    void loadEntities(dv::value map);
    void loadEntity(const dv::value& map);
    void loadEntity(const dv::value& map, Entity entity);
    void onSave(const Entity& entity);
    bool hasBlockingInside(AABB aabb);
    std::vector<Entity> getAllInside(AABB aabb);
    std::vector<Entity> getAllInRadius(glm::vec3 center, float radius);
    void despawn(entityid_t id);
    void despawn(std::vector<Entity> entities);
    dv::value serialize(const std::vector<Entity>& entities);

    void setNextID(entityid_t id) {
        nextID = id;
    }

    inline size_t size() const {
        return entities.size();
    }

    inline entityid_t peekNextID() const {
        return nextID;
    }
};
