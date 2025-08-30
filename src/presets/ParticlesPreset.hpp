#pragma once

#include <glm/vec3.hpp>
#include <string>
#include <vector>

#include "interfaces/Serializable.hpp"
#include "util/EnumMetadata.hpp"

enum class ParticleSpawnShape {
    /// @brief Coordinates are regulary distributed within 
    /// the volume of a ball.
    BALL = 0,
    /// @brief Coordinates are regulary distributed on 
    /// a sphere.
    SPHERE,
    /// @brief Coordinates are uniform distributed within 
    /// the volume of a box.
    BOX
};

VC_ENUM_METADATA(ParticleSpawnShape)
    {"ball", ParticleSpawnShape::BALL},
    {"sphere", ParticleSpawnShape::SPHERE},
    {"box", ParticleSpawnShape::BOX},
VC_ENUM_END

struct ParticlesPreset : public Serializable {
    /// @brief Collision detection
    bool collision = true;
    /// @brief Apply lighting
    bool lighting = true;
    /// @brief Use global up vector instead of camera-dependent one
    bool globalUpVector = false;
    /// @brief Max distance of actually spawning particles.
    float maxDistance = 32.0f;
    /// @brief Particles spawn interval
    float spawnInterval = 0.1f;
    /// @brief Particle life time
    float lifetime = 5.0f;
    /// @brief Life time spread divided by lifetime
    float lifetimeSpread = 0.2f;
    /// @brief Initial velocity
    glm::vec3 velocity {};
    /// @brief Velocity acceleration
    glm::vec3 acceleration {0.0f, -16.0f, 0.0f};
    /// @brief Random velocity magnitude applying to spawned particles.
    glm::vec3 explosion {2.0f};
    /// @brief Particle size
    glm::vec3 size {0.1f};
    /// @brief Particles size spread
    float sizeSpread = 0.2f;
    /// @brief Random initial angle spread
    float angleSpread = 0.0f;
    /// @brief Minimum angular velocity
    float minAngularVelocity = 0.0f;
    /// @brief Maximum angular velocity
    float maxAngularVelocity = 0.0f;
    /// @brief Spawn spread shape
    ParticleSpawnShape spawnShape = ParticleSpawnShape::BALL;
    /// @brief Spawn spread
    glm::vec3 spawnSpread {};
    /// @brief Texture name
    std::string texture = "";
    /// @brief Size of random sub-uv region
    float randomSubUV = 1.0f;
    /// @brief Animation frames
    std::vector<std::string> frames;

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};
