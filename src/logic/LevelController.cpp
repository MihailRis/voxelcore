#include "LevelController.hpp"

#include <algorithm>

#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "world/files/WorldFiles.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "objects/Players.hpp"
#include "objects/Player.hpp"
#include "physics/Hitbox.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Pathfinding.hpp"
#include "scripting/scripting.hpp"
#include "lighting/Lighting.hpp"
#include "settings.hpp"
#include "world/LevelEvents.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"

static debug::Logger logger("level-control");

LevelController::LevelController(
    Engine* engine, std::unique_ptr<Level> levelPtr, Player* clientPlayer
)
    : settings(engine->getSettings()),
      level(std::move(levelPtr)),
      chunks(std::make_unique<ChunksController>(*level)),
      playerTickClock(20, 3) {
    
    level->events->listen(LevelEventType::CHUNK_PRESENT, [](auto, Chunk* chunk) {
        scripting::on_chunk_present(*chunk, chunk->flags.loaded);
    });
    level->events->listen(LevelEventType::CHUNK_UNLOAD, [](auto, Chunk* chunk) {
        scripting::on_chunk_remove(*chunk);
    });

    if (clientPlayer) {
        chunks->lighting = std::make_unique<Lighting>(
            level->content, *clientPlayer->chunks
        );
    }
    blocks = std::make_unique<BlocksController>(
        *level, chunks ? chunks->lighting.get() : nullptr
    );
    scripting::on_world_load(this);

    // TODO: do something to players added later
    int confirmed;
    do {
        confirmed = 0;
        for (const auto& [_, player] : *level->players) {
            if (!player->isLoadingChunks()) {
                confirmed++;
                continue;
            }
            glm::vec3 position = player->getPosition();
            player->chunks->configure(
                std::floor(position.x), std::floor(position.z), 1
            );
            chunks->update(16, 1, 0, *player);
            if (player->chunks->get(
                    std::floor(position.x), 0, std::floor(position.z)
                )) {
                confirmed++;
            }
        }
    } while (confirmed < level->players->size());
}

void LevelController::update(float delta, bool pause) {
    level->pathfinding->performAllAsync(
        settings.pathfinding.stepsPerAsyncAgent.get()
    );
    for (const auto& [_, player] : *level->players) {
        if (player->isSuspended()) {
            continue;
        }
        player->rotationInterpolation.updateTimer(delta);
        player->updateEntity();
        glm::vec3 position = player->getPosition();
        player->chunks->configure(
            glm::floor(position.x),
            glm::floor(position.z),
            settings.chunks.loadDistance.get() + settings.chunks.padding.get()
        );
        chunks->update(
            settings.chunks.loadSpeed.get(),
            settings.chunks.loadDistance.get(),
            settings.chunks.padding.get(),
            *player
        );
    }
    if (!pause) {
        // update all objects that needed
        blocks->update(delta, settings.chunks.padding.get());
        level->entities->update(delta);
        for (const auto& [_, player] : *level->players) {
            if (player->isSuspended()) {
                continue;
            }
            if (playerTickClock.update(delta)) {
                if (player->getId() % playerTickClock.getParts() ==
                    playerTickClock.getPart()) {
                    
                    const auto& position = player->getPosition();
                    if (player->chunks->get(
                        std::floor(position.x),
                        std::floor(position.y),
                        std::floor(position.z)
                    )){
                        scripting::on_player_tick(
                            player.get(), playerTickClock.getTickRate()
                        );
                    }
                }
            }
        }
    }
    level->entities->clean();
}

void LevelController::saveWorld() {
    auto world = level->getWorld();
    if (world->isNameless()) {
        logger.info() << "nameless world will not be saved";
        return;
    }
    logger.info() << "writing world '" << world->getName() << "'";
    world->wfile->createDirectories();
    scripting::on_world_save();
    level->onSave();
    level->getWorld()->write(level.get());
}

void LevelController::onWorldQuit() {
    scripting::on_world_quit();
}

Level* LevelController::getLevel() {
    return level.get();
}

BlocksController* LevelController::getBlocksController() {
    return blocks.get();
}

ChunksController* LevelController::getChunksController() {
    return chunks.get();
}
