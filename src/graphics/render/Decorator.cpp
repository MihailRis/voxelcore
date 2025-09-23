#include "Decorator.hpp"

#include "ParticlesRenderer.hpp"
#include "WorldRenderer.hpp"
#include "TextsRenderer.hpp"
#include "TextNote.hpp"
#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "content/Content.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Block.hpp"
#include "world/Level.hpp"
#include "window/Camera.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "objects/Entities.hpp"
#include "objects/Entity.hpp"
#include "logic/LevelController.hpp"
#include "util/stringutil.hpp"
#include "engine/Engine.hpp"
#include "io/io.hpp"
#include "audio/audio.hpp"
#include "maths/util.hpp"

namespace fs = std::filesystem;

/// @brief Not greather than 64 for this BIG_PRIME value
inline constexpr int UPDATE_AREA_DIAMETER = 32;
/// @brief Number of blocks in the volume
inline constexpr int UPDATE_BLOCKS =
    UPDATE_AREA_DIAMETER * UPDATE_AREA_DIAMETER * UPDATE_AREA_DIAMETER;
/// @brief Number of update iterations
inline constexpr int ITERATIONS = 512;
/// @brief Big prime number used for pseudo-random 3d array iteration
inline constexpr int BIG_PRIME = 666667;

Decorator::Decorator(
    Engine& engine,
    LevelController& controller,
    WorldRenderer& renderer,
    const Assets& assets,
    Player& player
)
    : level(*controller.getLevel()),
      assets(assets),
      player(player),
      renderer(renderer) {
    controller.getBlocksController()->listenBlockInteraction(
    [this](auto player, const auto& pos, const auto& def, BlockInteraction type) {
        if (type == BlockInteraction::placing && def.particles) {
            addParticles(def, pos);
        }
    });
    for (const auto& [id, player] : *level.players) {
        if (id == this->player.getId()) {
            continue;
        }
        playerTexts[id] = renderer.texts->add(std::make_unique<TextNote>(
            util::str2wstr_utf8(player->getName()),
            playerNamePreset,
            player->getPosition()
        ));
    }
    playerNamePreset.deserialize(engine.getResPaths().readCombinedObject(
        "presets/text3d/player_name.toml"
    ));
}

void Decorator::addParticles(const Block& def, const glm::ivec3& pos) {
    const auto& found = blockEmitters.find(pos);
    if (found == blockEmitters.end()) {
        auto treg = util::get_texture_region(
            assets, def.particles->texture, ""
        );
        blockEmitters[pos] = renderer.particles->add(std::make_unique<Emitter>(
            level,
            glm::vec3{pos.x + 0.5, pos.y + 0.5, pos.z + 0.5},
            *def.particles,
            treg.texture,
            treg.region,
            -1
        ));
    }
}

void Decorator::updateRandom(
    float delta,
    const glm::ivec3& areaCenter,
    const WeatherPreset& weather
) {
    util::PseudoRandom random(rand());

    const auto& chunks = *player.chunks;
    const auto& indices = *level.content.getIndices();
    const auto& rainSplash = weather.fall.splash;

    auto pos = areaCenter + glm::ivec3(
        random.rand32() % 12,
        random.rand32() % 12,
        random.rand32() % 12
    );
    auto vox = chunks.get(pos);
    auto chunk = chunks.getChunkByVoxel(pos);
    if (vox == nullptr || chunk == nullptr) {
        return;
    }

    const auto& def = indices.blocks.require(vox->id);
    auto dst2 = util::distance2(pos, areaCenter);
    if (!def.obstacle || dst2 >= 256 || weather.fall.noise.empty()) {
        return;
    }
    for (int y = pos.y + 1; y < chunk->top; y++) {
        if (indices.blocks.require(chunks.get(pos.x, y, pos.z)->id).obstacle) {
            return;
        }
    }
    float intensity = weather.intensity * weather.fall.maxIntensity;
    if (rainSplash.has_value() && dst2 < 128 &&
        random.randFloat() < glm::pow(intensity, 2.0f)) {
        auto treg = util::get_texture_region(
            assets, "particles:rain_splash_0", ""
        );
        renderer.particles->add(std::make_unique<Emitter>(
            level,
            glm::vec3 {
                pos.x + random.randFloat(),
                pos.y + 1.1,
                pos.z + random.randFloat()},
            *rainSplash,
            treg.texture,
            treg.region,
            2
        ));
    }
    if (random.rand() % 200 < 3 && pos.y < areaCenter.y + 1) {
        auto sound = assets.get<audio::Sound>(weather.fall.noise);
        audio::play(
            sound,
            pos,
            false,
            intensity * intensity,
            1.0f,
            false,
            audio::PRIORITY_LOW,
            audio::get_channel_index("ambient")
        );
    }
}

void Decorator::update(
    float delta,
    const glm::ivec3& areaStart,
    const glm::ivec3& areaCenter
) {
    int index = currentIndex;
    currentIndex = (currentIndex + BIG_PRIME) % UPDATE_BLOCKS;

    const auto& chunks = *player.chunks;
    const auto& indices = *level.content.getIndices();

    int lx = index % UPDATE_AREA_DIAMETER;
    int lz = (index / UPDATE_AREA_DIAMETER) % UPDATE_AREA_DIAMETER;
    int ly = (index / UPDATE_AREA_DIAMETER / UPDATE_AREA_DIAMETER);
    
    glm::ivec3 offset {lx, ly, lz};
    auto pos = areaStart + offset;

    if (auto vox = chunks.get(pos)) {
        const auto& def = indices.blocks.require(vox->id);
        if (def.particles) {
            addParticles(def, pos);
        }
    }
}

void Decorator::updateBlockEmitters(const Camera& camera) {
    const auto& chunks = *player.chunks;
    const auto& indices = *level.content.getIndices();
    auto iter = blockEmitters.begin();
    while (iter != blockEmitters.end()) {
        auto emitter = renderer.particles->getEmitter(iter->second);
        if (emitter == nullptr) {
            iter = blockEmitters.erase(iter);
            continue;
        }

        bool remove = false;
        if (auto vox = chunks.get(iter->first)) {
            const auto& def = indices.blocks.require(vox->id);
            if (def.particles == nullptr) {
                remove = true;
            }
        } else {
            iter = blockEmitters.erase(iter);
            continue;
        }
        if (util::distance2(iter->first, glm::ivec3(camera.position)) >
            UPDATE_AREA_DIAMETER * UPDATE_AREA_DIAMETER) {
            remove = true;
        }
        if (remove) {
            emitter->stop();
            iter = blockEmitters.erase(iter);
            continue;
        }
        iter++;
    }
}

void Decorator::updateTextNotes() {
    for (const auto& [id, player] : *level.players) {
        if (id == this->player.getId() ||
            playerTexts.find(id) != playerTexts.end()) {
            continue;
        }
        playerTexts[id] = renderer.texts->add(std::make_unique<TextNote>(
            util::str2wstr_utf8(player->getName()),
            playerNamePreset,
            player->getPosition()
        ));
    }

    auto textsIter = playerTexts.begin();
    while (textsIter != playerTexts.end()) {
        auto note = renderer.texts->get(textsIter->second);
        auto player = level.players->get(textsIter->first);
        if (player == nullptr) {
            renderer.texts->remove(textsIter->second);
            textsIter = playerTexts.erase(textsIter);
        } else {
            glm::vec3 position = player->getPosition();
            if (auto entity = level.entities->get(player->getEntity())) {
                position = entity->getInterpolatedPosition();
            }
            note->setPosition(position + glm::vec3(0, 1, 0));
            ++textsIter;
        }
    }
}

void Decorator::updateRandomSounds(float delta, const Weather& weather) {
    thunderTimer += delta;
    util::PseudoRandom random(rand());
    if (thunderTimer >= 1.0f) {
        thunderTimer = 0.0f;
        if (random.randFloat() < weather.thunderRate()) {
            audio::play(
                assets.get<audio::Sound>("ambient/thunder"),
                glm::vec3(),
                false,
                1.0f,
                1.0f + random.randFloat() - 0.5f,
                false,
                audio::PRIORITY_NORMAL,
                audio::get_channel_index("ambient")
            );
        }
    }
}

void Decorator::update(
    float delta,
    const Camera& camera,
    const Weather& weather
) {
    updateRandomSounds(delta, weather);    

    glm::ivec3 pos = camera.position;
    for (int i = 0; i < ITERATIONS; i++) {
        update(delta, pos - glm::ivec3(UPDATE_AREA_DIAMETER / 2), pos);
    }
    int randIters = std::min(50'000, static_cast<int>(delta * 24'000));
    for (int i = 0; i < randIters; i++) {
        if (weather.a.intensity > 1.e-3f) {
            updateRandom(delta, pos, weather.a);
        }
        if (weather.b.intensity > 1.e-3f) {
            updateRandom(delta, pos, weather.b);
        }
    }
    updateBlockEmitters(camera);
    updateTextNotes();
}
