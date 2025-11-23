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
#include "voxels/Block.hpp"
#include "world/Level.hpp"
#include "engine/Engine.hpp"

LevelFrontend::LevelFrontend(
    Engine& engine,
    Player* currentPlayer,
    LevelController* controller,
    const EngineSettings& settings
)
    : level(*controller->getLevel()),
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

    auto& rassets = assets;
    controller->getBlocksController()->listenBlockInteraction(
        [currentPlayer, controller, &rassets](auto player, const auto& pos, const auto& def, BlockInteraction type) {
            const auto& level = *controller->getLevel();
            auto material = level.content.findBlockMaterial(def.material);
            if (material == nullptr) {
                return;
            }

            if (type == BlockInteraction::step) {
                auto sound = rassets.get<audio::Sound>(material->stepsSound);
                glm::vec3 pos {};
                auto soundsCamera = currentPlayer->currentCamera.get();
                if (currentPlayer->isCurrentCameraBuiltin()) {
                    soundsCamera = currentPlayer->fpCamera.get();
                }
                bool relative = player == currentPlayer && 
                    soundsCamera == currentPlayer->fpCamera.get();
                if (!relative) {
                    pos = player->getPosition();
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
            } else {
                audio::Sound* sound = nullptr;
                switch (type) {
                    case BlockInteraction::placing:
                        sound = rassets.get<audio::Sound>(material->placeSound);
                        break;
                    case BlockInteraction::destruction:
                        sound = rassets.get<audio::Sound>(material->breakSound);
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

LevelController* LevelFrontend::getController() const {
    return controller;
}
