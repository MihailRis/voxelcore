#pragma once

#include "typedefs.hpp"
#include "data/dv_fwd.hpp"

#include <string>
#include <entt/fwd.hpp>
#include <glm/vec3.hpp>

class Entities;
struct EntityDef;
struct Transform;
struct Rigidbody;
struct ScriptComponents;

inline std::string COMP_TRANSFORM = "transform";
inline std::string COMP_RIGIDBODY = "rigidbody";
inline std::string COMP_SKELETON = "skeleton";

namespace rigging {
    struct Skeleton;
    class SkeletonConfig;
}

struct EntityId {
    entityid_t uid;
    const EntityDef& def;
    bool destroyFlag = false;
    int64_t player = -1;
};

class Entity {
    Entities& entities;
    entityid_t id;
    entt::registry& registry;
    const entt::entity entity;
public:
    Entity(
        Entities& entities,
        entityid_t id,
        entt::registry& registry,
        const entt::entity entity
    )
        : entities(entities), id(id), registry(registry), entity(entity) {
    }

    dv::value serialize() const;

    EntityId& getID() const;

    bool isValid() const;

    const EntityDef& getDef() const;

    Transform& getTransform() const;

    Rigidbody& getRigidbody() const;

    ScriptComponents& getScripting() const;

    rigging::Skeleton& getSkeleton() const;

    void setRig(const rigging::SkeletonConfig* rigConfig);

    entityid_t getUID() const;

    int64_t getPlayer() const;

    void setPlayer(int64_t id);

    void setInterpolatedPosition(const glm::vec3& position);

    glm::vec3 getInterpolatedPosition() const;

    entt::entity getHandler() const {
        return entity;
    }

    void destroy();
};
