#include "Emitter.hpp"

#include <glm/gtc/random.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include "window/Camera.hpp"
#include "graphics/core/Texture.hpp"
#include "objects/Entities.hpp"
#include "objects/Entity.hpp"
#include "world/Level.hpp"

Emitter::Emitter(
    const Level& level,
    std::variant<glm::vec3, entityid_t> origin,
    ParticlesPreset preset,
    const Texture* texture,
    const UVRegion& region,
    int count
)
    : level(level),
      origin(std::move(origin)),
      prototype({this, 0, {}, preset.velocity, preset.lifetime, region, 0, 0}),
      texture(texture),
      count(count),
      preset(std::move(preset)) {
    random.setSeed(reinterpret_cast<ptrdiff_t>(this));
    this->prototype.emitter = this;
    timer = preset.spawnInterval * random.randFloat();
}

const Texture* Emitter::getTexture() const {
    return texture;
}

static inline glm::vec3 generate_coord(ParticleSpawnShape shape) {
    switch (shape) {
        case ParticleSpawnShape::BALL:
            return glm::ballRand(1.0f);
        case ParticleSpawnShape::SPHERE:
            return glm::sphericalRand(1.0f);
        case ParticleSpawnShape::BOX:
            return glm::linearRand(glm::vec3(-1.0f), glm::vec3(1.0f));
        default:
            return {};
    }
}

void Emitter::update(
    float delta,
    const glm::vec3& cameraPosition,
    std::vector<Particle>& particles
) {
    const float spawnInterval = preset.spawnInterval;
    if (count == 0 || (count == -1 && spawnInterval < FLT_EPSILON)) {
        return;
    }
    glm::vec3 position {};
    if (auto staticPos = std::get_if<glm::vec3>(&origin)) {
        position = *staticPos;
    } else if (auto entityId = std::get_if<entityid_t>(&origin)) {
        if (auto entity = level.entities->get(*entityId)) {
            position = entity->getTransform().pos;
        } else {
            stop();
            return;
        }
    }
    timer += delta;
    const float maxDistance = preset.maxDistance;
    if (glm::distance2(position, cameraPosition) > maxDistance * maxDistance) {
        if (count > 0) {
            if (spawnInterval < FLT_EPSILON) {
                count = 0;
                return;
            }
            int skipped = timer / spawnInterval;
            count = std::max(0, count - skipped);
            timer -= skipped * spawnInterval;
        }
        if (count < 0) {
            int skipped = timer / spawnInterval;
            timer -= skipped * spawnInterval;
        }
        return;
    }
    while (count && timer > spawnInterval) {
        // spawn particle
        Particle particle = prototype;
        particle.emitter = this;
        particle.random = random.rand32();
        if (glm::abs(preset.angleSpread) >= 0.005f) {
            particle.angle =
                random.randFloat() * preset.angleSpread * glm::pi<float>() * 2;
        }
        particle.angularVelocity =
            (preset.minAngularVelocity +
            random.randFloat() *
                (preset.maxAngularVelocity - preset.minAngularVelocity)) *
                ((random.rand() % 2) * 2 - 1);

        glm::vec3 spawnOffset = generate_coord(preset.spawnShape);
        spawnOffset *= preset.spawnSpread;

        particle.position = position + spawnOffset;
        particle.lifetime *= 1.0f - preset.lifetimeSpread * random.randFloat();
        particle.velocity += glm::ballRand(1.0f) * preset.explosion;
        if (preset.randomSubUV < 1.0f) {
            particle.region.autoSub(
                preset.randomSubUV,
                preset.randomSubUV,
                random.randFloat(),
                random.randFloat()
            );
        }
        particles.push_back(std::move(particle));
        timer -= spawnInterval;
        if (count > 0) {
            count--;
        }
        refCount++;
    }
}

void Emitter::stop() {
    count = 0;
}

bool Emitter::isDead() const {
    return count == 0;
}

bool Emitter::isReferred() const {
    return refCount > 0;
}

const EmitterOrigin& Emitter::getOrigin() const {
    return origin;
}

void Emitter::setOrigin(const EmitterOrigin& origin) {
    this->origin = origin;
}
