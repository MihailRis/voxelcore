#pragma once

#include <vector>
#include <memory>
#include <unordered_map>

#include "Emitter.hpp"
#include "typedefs.hpp"

class Texture;
class Assets;
class Camera;
class Chunks;
class MainBatch;
class Level;
struct GraphicsSettings;

class ParticlesRenderer {
    const Chunks& chunks;
    const Assets& assets;
    const GraphicsSettings& settings;
    std::unordered_map<const Texture*, std::vector<Particle>> particles;
    std::unique_ptr<MainBatch> batch;

    std::unordered_map<u64id_t, std::unique_ptr<Emitter>> emitters;
    u64id_t nextEmitter = 1;

    void renderParticle(
        Particle& particle, const Camera& camera, bool backlight
    );
    void updateParticles(float delta);
public:
    ParticlesRenderer(
        const Assets& assets,
        const Level& level,
        const Chunks& chunks,
        const GraphicsSettings& settings
    );
    ~ParticlesRenderer();

    void update(const Camera& camera, float delta);
    void render(const Camera& camera);

    u64id_t add(std::unique_ptr<Emitter> emitter);

    /// @brief Get emitter by UID
    /// @return Emitter or nullptr
    Emitter* getEmitter(u64id_t id) const;

    static size_t visibleParticles;
    static size_t aliveEmitters;
};
