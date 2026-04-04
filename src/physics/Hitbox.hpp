#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include "maths/aabb.hpp"
#include "typedefs.hpp"
#include "util/EnumMetadata.hpp"

#include <set>
#include <string>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

enum class SensorType {
    AABB,
    RADIUS,
};

union SensorParams {
    AABB aabb;
    glm::vec4 radial; // x,y,z calculated entity coords + w - radius

    constexpr SensorParams() : aabb() {
    }
};

using sensorcallback = std::function<void(entityid_t, size_t, entityid_t)>;

struct Sensor {
    bool enabled = true;
    SensorType type;
    size_t index;
    entityid_t entity;
    SensorParams params;
    SensorParams calculated;
    std::set<entityid_t> prevEntered;
    std::set<entityid_t> nextEntered;
    sensorcallback enterCallback;
    sensorcallback exitCallback;
};

enum class BodyType {
    STATIC, KINEMATIC, DYNAMIC
};

VC_ENUM_METADATA(BodyType)
    {"static", BodyType::STATIC},
    {"kinematic", BodyType::KINEMATIC},
    {"dynamic", BodyType::DYNAMIC},
VC_ENUM_END

struct Hitbox {
    entityid_t entity;
    BodyType type;
    glm::vec3 position;
    glm::vec3 halfsize;
    glm::vec3 velocity;
    glm::vec3 scale {1.0f, 1.0f, 1.0f};
    float linearDamping = 0.5;
    float verticalDamping = 1.0f;
    bool grounded = false;
    float gravityScale = 1.0f;
    bool crouching = false;
    float stepHeight = 0.5f;
    float mass = 1.0f;
    float elasticity = 0.0f;
    std::string material;
    std::string groundMaterial;
    glm::vec3 groundVelocity {};
    
    glm::vec3 prevPosition {};
    glm::vec3 prevVelocity {};
    bool prevGrounded = false;

    static inline constexpr float TELEPORT_THRESOLD_SQR = 0.5f;

    Hitbox(
        entityid_t entity, BodyType type, glm::vec3 position, glm::vec3 halfsize
    );

    AABB getAABB() const {
        return AABB(position - halfsize, position + halfsize);
    }

    glm::vec3 getHalfSize() const {
        return halfsize * scale;
    }

    void setPos(const glm::vec3& vec) {
        position = vec;
        if (glm::distance2(position, prevPosition) >= TELEPORT_THRESOLD_SQR) {
            prevPosition = vec;
        }
    }

    glm::vec3 getSurfaceVelocity() const {
        return velocity - groundVelocity;
    }
};
