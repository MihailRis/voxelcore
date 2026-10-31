#pragma once

#include "typedefs.hpp"
#include "data/dv_fwd.hpp"
#include "Transform.hpp"
#include "Rigidbody.hpp"
#include "ScriptComponents.hpp"

#include <memory>
#include <string>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

class Entities;
struct EntityDef;

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
    EntityId id;
    Transform transform;
    Rigidbody rigidbody;
    ScriptComponents scripting;
    std::unique_ptr<rigging::Skeleton> skeleton;
public:
    Entity(
        Entities& entities,
        EntityId id,
        Transform transform,
        Rigidbody rigidbody,
        std::unique_ptr<rigging::Skeleton> skeleton
    );

    Entity(const Entity&) = delete;
    Entity& operator=(const Entity&) = delete;

    ~Entity();

    dv::value serialize() const;

    EntityId& getID() {
        return id;
    }
    const EntityId& getID() const {
        return id;
    }

    const EntityDef& getDef() const {
        return id.def;
    }

    Transform& getTransform() {
        return transform;
    }
    const Transform& getTransform() const {
        return transform;
    }

    Rigidbody& getRigidbody() {
        return rigidbody;
    }
    const Rigidbody& getRigidbody() const {
        return rigidbody;
    }

    ScriptComponents& getScripting() {
        return scripting;
    }
    const ScriptComponents& getScripting() const {
        return scripting;
    }

    rigging::Skeleton* getSkeleton() {
        return skeleton.get();
    }
    const rigging::Skeleton* getSkeleton() const {
        return skeleton.get();
    }

    void setRig(std::shared_ptr<const rigging::SkeletonConfig> rigConfig);

    entityid_t getUID() const {
        return id.uid;
    }

    int64_t getPlayer() const {
        return id.player;
    }

    void setPlayer(int64_t player);

    void setInterpolatedPosition(const glm::vec3& position);

    glm::vec3 getInterpolatedPosition() const;

    void setInterpolatedRotation(const glm::quat& rotation);

    void destroy();
};
