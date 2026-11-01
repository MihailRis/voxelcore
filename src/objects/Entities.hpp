#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "constants.hpp"
#include "Entity.hpp"
#include "physics/Hitbox.hpp"
#include "Transform.hpp"
#include "Rigidbody.hpp"
#include "ScriptComponents.hpp"
#include "typedefs.hpp"
#include "util/Clock.hpp"

#include <map>

struct EntityDef;

class Level;
class Assets;
class Entity;

namespace rigging {
    struct Skeleton;
    class SkeletonConfig;
}

class Entities final {
    Level& level;
    std::map<entityid_t, std::unique_ptr<Entity>> entities;
    entityid_t nextID = 1;
    util::Clock sensorsTickClock;
    util::Clock updateTickClock;
    Assets* assets = nullptr;

    void updateSensors(
        Rigidbody& body, const Transform& tsf, std::vector<Sensor*>& sensors
    );
    void preparePhysics(float delta);
public:
    Entities(Level& level);

    ~Entities();

    void setAssets(Assets& assets);

    void clean();
    void updatePhysics(float delta);
    void update(float delta);

    entityid_t spawn(
        const EntityDef& def,
        glm::vec3 position,
        dv::value args = nullptr,
        dv::value saved = nullptr,
        entityid_t uid = 0
    );

    Entity* get(entityid_t id);

    struct RaycastSettings {
        /// @param ignore Ignored entity uid
        entityid_t ignoredUid;
        /// @param solidEntitiesOnly If true, only entities with solid hitboxes 
        /// will be checked
        bool solidEntitiesOnly = false;
        bool entityFilterExcludeMode = false;
        const std::set<entitydefid_t>* entitiesFilter = nullptr;
        bool includeNonSelectable = false;
    };

    struct RaycastResult {
        entityid_t entity;
        glm::ivec3 normal;
        float distance;
    };

    /// @brief Entities raycast. No blocks check included, use combined with
    /// Chunks.rayCast
    /// @param start Ray start
    /// @param dir Ray direction normalized vector
    /// @param maxDistance Max ray length
    /// @return An optional structure containing entity, normal and distance
    std::optional<RaycastResult> rayCast(
        glm::vec3 start,
        glm::vec3 dir,
        float maxDistance,
        const RaycastSettings& settings
    );

    void loadEntities(dv::value map);
    void loadEntity(const dv::value& map);
    void loadEntity(const dv::value& map, Entity& entity);
    void onSave(const Entity& entity);
    bool hasBlockingInside(AABB aabb);
    std::vector<Entity*> getAllInside(AABB aabb);
    std::vector<Entity*> getAllInRadius(glm::vec3 center, float radius);
    void despawn(entityid_t id);
    void despawn(const std::vector<Entity*>& entities);
    dv::value serialize(const std::vector<Entity*>& entities);

    void setNextID(entityid_t id) {
        nextID = id;
    }

    inline const std::map<entityid_t, std::unique_ptr<Entity>>& getEntities(
    ) const {
        return entities;
    }

    inline size_t size() const {
        return entities.size();
    }

    inline entityid_t peekNextID() const {
        return nextID;
    }
};
