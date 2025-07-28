#include "ChunksController.hpp"

#include <limits.h>
#include <memory>

#include "content/Content.hpp"
#include "world/files/WorldFiles.hpp"
#include "graphics/core/Mesh.hpp"
#include "lighting/Lighting.hpp"
#include "maths/voxmaths.hpp"
#include "util/timeutil.hpp"
#include "objects/Player.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/GlobalChunks.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "world/generator/WorldGenerator.hpp"

const uint MAX_WORK_PER_FRAME = 128;
const uint MIN_SURROUNDING = 9;

ChunksController::ChunksController(Level& level)
    : level(level),
      generator(std::make_unique<WorldGenerator>(
          level.content.generators.require(level.getWorld()->getGenerator()),
          level.content,
          level.getWorld()->getSeed()
      )) {}

ChunksController::~ChunksController() = default;

void ChunksController::update(
    int64_t maxDuration, int loadDistance, uint padding, Player& player
) const {
    const auto& position = player.getPosition();
    int centerX = floordiv<CHUNK_W>(glm::floor(position.x));
    int centerY = floordiv<CHUNK_D>(glm::floor(position.z));
    
    if (player.isLoadingChunks()) {
        /// FIXME: one generator for multiple players
        generator->update(centerX, centerY, loadDistance);
    } else {
        return;
    }

    int64_t mcstotal = 0;

    for (uint i = 0; i < MAX_WORK_PER_FRAME; i++) {
        timeutil::Timer timer;
        if (loadVisible(player, padding)) {
            int64_t mcs = timer.stop();
            if (mcstotal + mcs < maxDuration * 1000) {
                mcstotal += mcs;
                continue;
            }
        }
        break;
    }
}

bool ChunksController::isInLoadingZone(
    const Player& player, uint padding, int x, int z
) const {
    const auto& chunks = *player.chunks;
    int sizeX = chunks.getWidth();
    int sizeY = chunks.getHeight();

    int minDistance = ((sizeX - padding * 2) / 2) * ((sizeY - padding * 2) / 2);

    int lx = (x - chunks.getOffsetX()) - sizeX / 2;
    int lz = (z - chunks.getOffsetY()) - sizeY / 2;
    int distance = (lx * lx + lz * lz);

    return distance < minDistance;
}

bool ChunksController::loadVisible(const Player& player, uint padding) const {
    auto& chunks = *player.chunks;
    int sizeX = chunks.getWidth();
    int sizeY = chunks.getHeight();

    int nearX = 0;
    int nearZ = 0;
    bool assigned = false;
    int minDistance = ((sizeX - padding * 2) / 2) * ((sizeY - padding * 2) / 2);
    int maxDistance = ((sizeX) / 2) * ((sizeY) / 2);
    for (uint z = 0; z < sizeY; z++) {
        for (uint x = 0; x < sizeX; x++) {
            int index = z * sizeX + x;
            int lx = x - sizeX / 2;
            int lz = z - sizeY / 2;
            int distance = (lx * lx + lz * lz);
            auto& chunk = chunks.getChunks()[index];
            if (chunk != nullptr) {
                if (distance >= maxDistance) {
                    chunks.remove(
                        x + chunks.getOffsetX(), z + chunks.getOffsetY()
                    );
                }
                continue;
            }
        }
    }
    for (uint z = padding; z < sizeY - padding; z++) {
        for (uint x = padding; x < sizeX - padding; x++) {
            int index = z * sizeX + x;
            int lx = x - sizeX / 2;
            int lz = z - sizeY / 2;
            int distance = (lx * lx + lz * lz);
            auto& chunk = chunks.getChunks()[index];
            if (chunk != nullptr) {
                if (chunk->flags.loaded && !chunk->flags.lighted) {
                    if (buildLights(player, chunk)) {
                        return true;
                    }
                }
                continue;
            }

            if (distance < minDistance) {
                minDistance = distance;
                nearX = x;
                nearZ = z;
                assigned = true;
            }
        }
    }

    const auto& chunk = chunks.getChunks()[nearZ * sizeX + nearX];
    if (chunk != nullptr || !assigned || !player.isLoadingChunks()) {
        return false;
    }
    int offsetX = chunks.getOffsetX();
    int offsetY = chunks.getOffsetY();
    createChunk(player, nearX + offsetX, nearZ + offsetY);
    return true;
}

bool ChunksController::buildLights(
    const Player& player, const std::shared_ptr<Chunk>& chunk
) const {
    int surrounding = 0;
    for (int oz = -1; oz <= 1; oz++) {
        for (int ox = -1; ox <= 1; ox++) {
            if (player.chunks->getChunk(chunk->x + ox, chunk->z + oz))
                surrounding++;
        }
    }
    if (surrounding == MIN_SURROUNDING) {
        if (lighting) {
            bool lightsCache = chunk->flags.loadedLights;
            if (!lightsCache) {
                lighting->buildSkyLight(chunk->x, chunk->z);
            }
            lighting->onChunkLoaded(chunk->x, chunk->z, !lightsCache);
        }
        chunk->flags.lighted = true;
        return true;
    }
    return false;
}

void ChunksController::createChunk(const Player& player, int x, int z) const {
    if (!player.isLoadingChunks()) {
        if (auto chunk = level.chunks->fetch(x, z)) {
            player.chunks->putChunk(chunk);
        }
        return;
    }
    auto chunk = level.chunks->create(x, z);
    player.chunks->putChunk(chunk);
    auto& chunkFlags = chunk->flags;

    if (!chunkFlags.loaded) {
        generator->generate(chunk->voxels, x, z);
        chunkFlags.unsaved = true;
    }
    chunk->updateHeights();

    if (!chunkFlags.loadedLights) {
        Lighting::prebuildSkyLight(*chunk, *level.content.getIndices());
    }
    chunkFlags.loaded = true;
    chunkFlags.ready = true;
}
