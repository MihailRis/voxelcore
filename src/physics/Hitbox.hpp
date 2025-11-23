#pragma once

#include "maths/aabb.hpp"
#include "typedefs.hpp"
#include "util/EnumMetadata.hpp"

#include <set>
#include <string>
#include <functional>
#include <glm/glm.hpp>

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
    BodyType type;
    glm::vec3 position;
    glm::vec3 halfsize;
    glm::vec3 velocity;
    float linearDamping = 0.5;
    float friction = 1.0f;
    float verticalDamping = 1.0f;
    bool grounded = false;
    float gravityScale = 1.0f;
    bool crouching = false;

    Hitbox(BodyType type, glm::vec3 position, glm::vec3 halfsize);

    AABB getAABB() const {
        return AABB(position-halfsize, position+halfsize);
    }
};
