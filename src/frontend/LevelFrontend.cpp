#include "LevelFrontend.hpp"

#include "ContentGfxCache.hpp"

#include "assets/Assets.hpp"
#include "audio/audio.hpp"
#include "content/Content.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/render/BlocksPreview.hpp"
#include "logic/LevelController.hpp"
#include "logic/PlayerController.hpp"
#include "objects/Player.hpp"
#include "physics/Hitbox.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunks.hpp"
#include "world/Level.hpp"
#include "engine/Engine.hpp"

LevelFrontend::LevelFrontend(
    Engine& engine,
    PlayerController& playerController,
    LevelController& controller,
    const EngineSettings& settings
)
    : level(*controller.getLevel()),
      controller(controller),
      assets(*engine.getAssets()),
      contentCache(std::make_unique<ContentGfxCache>(
          level.content, assets, settings.graphics
      )) {
    assets.store(
        BlocksPreview::build(
            engine.getWindow(),
            *contentCache,
            *engine.getAssets(),
            *level.content.getIndices()
        ),
        "block-previews"
    );

    auto& currentPlayer = playerController.getPlayer();
    auto& assets = this->assets;
    auto& level = this->level;

    playerController.setFootstepCallback(
        [&level, &currentPlayer, &assets](const Hitbox& hitbox) {
        const BlockMaterial* material = nullptr;
        if (hitbox.groundMaterial.empty()) {
            const auto& pos = hitbox.position;
            const auto& half = hitbox.getHalfSize();

            auto& blockIndices = level.content.getIndices()->blocks;
            for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
                for (int offsetX = -1; offsetX <= 1; offsetX++) {
                    int x = std::floor(pos.x + half.x * offsetX);
                    int y = std::floor(pos.y - half.y * 1.1f);
                    int z = std::floor(pos.z + half.z * offsetZ);
                    auto vox = currentPlayer.chunks->get(x, y, z);
                    if (vox) {
                        auto& def = blockIndices.require(vox->id);
                        if (!def.obstacle) {
                            continue;
                        }
                        material = level.content.findBlockMaterial(def.material);
                        break;
                    }
                }
            }
        } else {
            material = level.content.findBlockMaterial(hitbox.groundMaterial);
        }
        if (material == nullptr) {
            return;
        }

        auto sound = assets.get<audio::Sound>(material->stepsSound);
        glm::vec3 pos {};
        auto soundsCamera = currentPlayer.currentCamera.get();
        if (currentPlayer.isCurrentCameraBuiltin()) {
            soundsCamera = currentPlayer.fpCamera.get();
        }
        bool relative = soundsCamera == currentPlayer.fpCamera.get();
        if (!relative) {
            pos = currentPlayer.getPosition();
        }
        audio::play(
            sound, 
            pos, 
            relative, 
            0.333f, 
            1.0f + (rand() % 6 - 3) * 0.05f, 
            false,
            audio::PRIORITY_LOW,
            audio::get_channel_index("regular")
        );
    });

    controller.getBlocksController()->listenBlockInteraction(
        [&level, &assets]
        (auto player, const auto& pos, const auto& def, BlockInteraction type) {
            auto material = level.content.findBlockMaterial(def.material);
            if (material == nullptr) {
                return;
            }

            if (type != BlockInteraction::step) {
                audio::Sound* sound = nullptr;
                switch (type) {
                    case BlockInteraction::placing:
                        sound = assets.get<audio::Sound>(material->placeSound);
                        break;
                    case BlockInteraction::destruction:
                        sound = assets.get<audio::Sound>(material->breakSound);
                        break; 
                    default:
                        break;   
                }
                audio::play(
                    sound, 
                    glm::vec3(pos.x, pos.y, pos.z) + 0.5f, 
                    false, 
                    1.0f,
                    1.0f + (rand() % 6 - 3) * 0.05f, 
                    false,
                    audio::PRIORITY_NORMAL,
                    audio::get_channel_index("regular")
                );
            }
        }
    );
}

LevelFrontend::~LevelFrontend() = default;

Level& LevelFrontend::getLevel() {
    return level;
}

ContentGfxCache& LevelFrontend::getContentGfxCache() {
    return *contentCache;
}

const ContentGfxCache& LevelFrontend::getContentGfxCache() const {
    return *contentCache;
}

LevelController& LevelFrontend::getController() const {
    return controller;
}
