#include "ChunksController.hpp"

#include <algorithm>
#include <limits.h>
#include <memory>

#include "content/Content.hpp"
#include "world/files/WorldFiles.hpp"
#include "graphics/core/Mesh.hpp"
#include "lighting/Lighting.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Player.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/GlobalChunks.hpp"
#include "world/Level.hpp"
#include "world/LevelEvents.hpp"
#include "world/World.hpp"
#include "world/generator/WorldGenerator.hpp"
#include "debug/Logger.hpp"

const uint MAX_WORK_PER_FRAME = 128;
const uint MIN_SURROUNDING = 9;
const float WORK_UNITS_PER_SPEED_SECOND = 600.0f;
const float MAX_SECONDS_LIMIT = 0.25f;
const int CREATE_CHUNK_COST = 2;
const int BUILD_LIGHTS_COST = 4;

static debug::Logger logger("chunks-control");

ChunksController::ChunksController(Level& level)
    : level(level),
      generator(std::make_unique<WorldGenerator>(
          level.content.generators.require(level.environment.generator),
          level.content,
          level.getWorld().getSeed()
      )) {}

ChunksController::~ChunksController() = default;

void ChunksController::update(
    float delta,
    int loadSpeed,
    int loadDistance,
    uint padding,
    Player& player,
    bool isLocalPlayer
) {
    const auto& position = player.getPosition();
    int centerX = floordiv<CHUNK_W>(glm::floor(position.x));
    int centerY = floordiv<CHUNK_D>(glm::floor(position.z));
    
    if (player.isLoadingChunks()) {
        /// FIXME: one generator for multiple players
        generator->update(centerX, centerY, loadDistance);
    } else {
        return;
    }

    auto& chunks = *player.chunks;
    auto& state = states[player.getId()];
    syncState(state, chunks, padding);

    float rate = loadSpeed * WORK_UNITS_PER_SPEED_SECOND;
    state.budget = std::min(
        state.budget + rate * delta, rate * MAX_SECONDS_LIMIT
    );
    for (uint i = 0; i < MAX_WORK_PER_FRAME && state.budget > 0.0f; i++) {
        int cost = loadVisible(state, player, isLocalPlayer);
        if (cost == 0) {
            state.budget = 0.0f;
            break;
        }
        state.budget -= cost;
    }
    state.chunksCount = chunks.getChunksCount();
}

void ChunksController::syncState(
    LoadingState& state, Chunks& chunks, uint padding
) const {
    int sizeX = chunks.getWidth();
    int sizeY = chunks.getHeight();
    bool layoutChanged = state.width != sizeX || state.height != sizeY ||
                         state.padding != padding;
    bool moved = state.offsetX != chunks.getOffsetX() ||
                 state.offsetZ != chunks.getOffsetY();

    if (layoutChanged) {
        state.width = sizeX;
        state.height = sizeY;
        state.padding = padding;
        state.order.clear();

        int pad = static_cast<int>(padding);
        int minDistance = ((sizeX - pad * 2) / 2) * ((sizeY - pad * 2) / 2);
        std::vector<std::pair<int, int>> cells;
        for (int z = pad; z < sizeY - pad; z++) {
            for (int x = pad; x < sizeX - pad; x++) {
                int lx = x - sizeX / 2;
                int lz = z - sizeY / 2;
                int distance = lx * lx + lz * lz;
                if (distance < minDistance) {
                    cells.emplace_back(distance, z * sizeX + x);
                }
            }
        }
        std::sort(cells.begin(), cells.end());
        state.order.reserve(cells.size());
        for (const auto& [_, index] : cells) {
            state.order.push_back(index);
        }
    }
    if (layoutChanged || moved) {
        removeFarChunks(chunks);
        state.offsetX = chunks.getOffsetX();
        state.offsetZ = chunks.getOffsetY();
    }
    if (layoutChanged || moved || state.chunksCount != chunks.getChunksCount()) {
        state.loadIndex = 0;
        state.lightIndex = 0;
        state.lightsDirty = true;
        state.chunksCount = chunks.getChunksCount();
    }
}

void ChunksController::removeFarChunks(Chunks& chunks) const {
    int sizeX = chunks.getWidth();
    int sizeY = chunks.getHeight();
    int maxDistance = (sizeX / 2) * (sizeY / 2);
    const auto& buffer = chunks.getChunks();
    for (int z = 0; z < sizeY; z++) {
        for (int x = 0; x < sizeX; x++) {
            if (buffer[z * sizeX + x] == nullptr) {
                continue;
            }
            int lx = x - sizeX / 2;
            int lz = z - sizeY / 2;
            if (lx * lx + lz * lz >= maxDistance) {
                chunks.remove(x + chunks.getOffsetX(), z + chunks.getOffsetY());
            }
        }
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

int ChunksController::loadVisible(
    LoadingState& state, const Player& player, bool isLocalPlayer
) const {
    auto& chunks = *player.chunks;
    const auto& buffer = chunks.getChunks();
    const auto& order = state.order;

    while (state.loadIndex < order.size() &&
           buffer[order[state.loadIndex]] != nullptr) {
        state.loadIndex++;
    }

    if (isLocalPlayer && state.lightsDirty) {
        while (state.lightIndex < state.loadIndex) {
            const auto& chunk = buffer[order[state.lightIndex]];
            if (chunk && chunk->flags.loaded && !chunk->flags.lighted) {
                break;
            }
            state.lightIndex++;
        }
        for (size_t i = state.lightIndex; i < state.loadIndex; i++) {
            const auto& chunk = buffer[order[i]];
            if (chunk && chunk->flags.loaded && !chunk->flags.lighted &&
                buildLights(player, chunk)) {
                return BUILD_LIGHTS_COST;
            }
        }
        state.lightsDirty = false;
    }

    if (state.loadIndex >= order.size() || !player.isLoadingChunks()) {
        return 0;
    }
    int index = order[state.loadIndex];
    int sizeX = chunks.getWidth();
    createChunk(
        player,
        index % sizeX + chunks.getOffsetX(),
        index / sizeX + chunks.getOffsetY()
    );
    state.loadIndex++;
    state.lightsDirty = true;
    return CREATE_CHUNK_COST;
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
        if (lighting && chunk->lightmap) {
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
    auto chunk = level.chunks->create(x, z, lighting != nullptr);
    player.chunks->putChunk(chunk);
    auto& chunkFlags = chunk->flags;
    if (!chunkFlags.loaded) {
        generator->generate(chunk->voxels, x, z);
        chunkFlags.unsaved = true;
    }
    chunk->updateHeights();
    level.events->trigger(LevelEventType::CHUNK_PRESENT, chunk.get());
    if (!chunkFlags.loadedLights && chunk->lightmap) {
        Lighting::prebuildSkyLight(*chunk, *level.content.getIndices());
    }
    chunkFlags.loaded = true;
    chunkFlags.ready = true;
}
