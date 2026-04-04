#include "BlocksController.hpp"

#include "content/Content.hpp"
#include "items/Inventories.hpp"
#include "items/Inventory.hpp"
#include "lighting/Lighting.hpp"
#include "maths/fastmaths.hpp"
#include "scripting/scripting.hpp"
#include "util/timeutil.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/voxel.hpp"
#include "voxels/blocks_agent.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "util/random.hpp"

#include <random>

static inline constexpr int CHUNK_RANDOM_TICK_SEGMENTS = 4;

BlocksController::BlocksController(const Level& level, Lighting* lighting)
    : level(level),
      chunks(*level.chunks),
      lighting(lighting),
      randTickClock(20, 3),
      blocksTickClock(20, 3),
      worldTickClock(20, 1) {
}

void BlocksController::updateSides(int x, int y, int z) {
    updateBlock(x - 1, y, z);
    updateBlock(x + 1, y, z);
    updateBlock(x, y - 1, z);
    updateBlock(x, y + 1, z);
    updateBlock(x, y, z - 1);
    updateBlock(x, y, z + 1);
}

void BlocksController::updateSides(int x, int y, int z, int w, int h, int d) {
    voxel* vox = blocks_agent::get(chunks, x, y, z);
    const auto& def = level.content.getIndices()->blocks.require(vox->id);
    const auto& rot = def.rotations.variants[vox->state.rotation];
    const auto& xaxis = rot.axes[0];
    const auto& yaxis = rot.axes[1];
    const auto& zaxis = rot.axes[2];
    for (int ly = -1; ly <= h; ly++) {
        for (int lz = -1; lz <= d; lz++) {
            for (int lx = -1; lx <= w; lx++) {
                if (lx >= 0 && lx < w && ly >= 0 && ly < h && lz >= 0 && lz < d) {
                    continue;
                }
                updateBlock(
                    x + lx * xaxis.x + ly * yaxis.x + lz * zaxis.x,
                    y + lx * xaxis.y + ly * yaxis.y + lz * zaxis.y,
                    z + lx * xaxis.z + ly * yaxis.z + lz * zaxis.z
                );
            }
        }
    }
}

void BlocksController::breakBlock(
    Player* player, const Block& def, int x, int y, int z
) {
    onBlockInteraction(
        player, glm::ivec3(x, y, z), def, BlockInteraction::destruction
    );
    blocks_agent::set(chunks, x, y, z, 0, {});
    if (lighting) {
        lighting->onBlockSet(x, y, z, 0);
    }
    scripting::on_block_broken(player, def, glm::ivec3(x, y, z));
    if (def.rt.extended) {
        updateSides(x, y, z , def.size.x, def.size.y, def.size.z);
    } else {
        updateSides(x, y, z);
    }
}

void BlocksController::placeBlock(
    Player* player, const Block& def, blockstate state, int x, int y, int z
) {
    auto voxel = blocks_agent::get(chunks, x, y, z);
    if (voxel == nullptr) {
        return;
    }
    const auto& prevDef = level.content.getIndices()->blocks.require(voxel->id);
    scripting::on_block_replaced(player, prevDef, {x, y, z});

    onBlockInteraction(
        player, glm::ivec3(x, y, z), def, BlockInteraction::placing
    );
    blocks_agent::set(chunks, x, y, z, def.rt.id, state);
    if (lighting) {
        lighting->onBlockSet(x, y, z, def.rt.id);
    }
    scripting::on_block_placed(player, def, glm::ivec3(x, y, z));
    if (def.rt.extended) {
        updateSides(x, y, z , def.size.x, def.size.y, def.size.z);
    } else {
        updateSides(x, y, z);
    }
}

void BlocksController::updateBlock(int x, int y, int z) {
    voxel* vox = blocks_agent::get(chunks, x, y, z);
    if (vox == nullptr) return;
    const auto& def = level.content.getIndices()->blocks.require(vox->id);
    if (def.grounded) {
        if (def.rt.extended) {
            auto origin = blocks_agent::seek_origin(chunks, {x, y, z}, def, vox->state);
            if (!blocks_agent::check_grounding(chunks, def, vox->state.rotation, origin)) {
                breakBlock(nullptr, def, origin.x, origin.y, origin.z);
                return;
            }
        } else {
            if (!blocks_agent::check_grounding(chunks, def, vox->state.rotation, {x, y, z})) {
                breakBlock(nullptr, def, x, y, z);
                return;
            }        
        }
    }
    if (def.rt.funcsset.update) {
        scripting::update_block(def, glm::ivec3(x, y, z));
    }
}

void BlocksController::update(float delta, uint padding) {
    if (randTickClock.update(delta)) {
        randomTick(randTickClock.getPart(), randTickClock.getParts(), padding);
    }
    if (blocksTickClock.update(delta)) {
        onBlocksTick(blocksTickClock.getTickId(), blocksTickClock.getParts());
    }
    if (worldTickClock.update(delta)) {
        scripting::on_world_tick(worldTickClock.getTickRate());
    }
}

void BlocksController::onBlocksTick(int tickid, int parts) {
    const auto& indices = level.content.getIndices()->blocks;
    int tickRate = blocksTickClock.getTickRate();
    for (size_t id = 0; id < indices.count(); id++) {
        if ((id + tickid) % parts != 0) continue;
        auto& def = indices.require(id);
        auto interval = def.tickInterval;
        if (def.rt.funcsset.onblockstick && tickid / parts % interval == 0) {
            scripting::on_blocks_tick(def, tickRate / interval);
        }
    }
}

void BlocksController::randomTick(
    const Chunk& chunk, const ContentIndices* indices
) {
    const int segments = CHUNK_RANDOM_TICK_SEGMENTS;
    const int segheight = CHUNK_H / segments;

    static std::array<int, CHUNK_VOL / segments> randomPattern;
    static bool randomPatternInitialized = false;
    if (!randomPatternInitialized) {
        randomPatternInitialized = true;
        for (int i = 0; i < randomPattern.size(); i++) {
            randomPattern[i] = i;
        }
        auto randomDevice = std::random_device{};
        auto randomEngine =
            util::seeded_random_engine<std::mt19937_64>(randomDevice);
        std::shuffle(randomPattern.begin(), randomPattern.end(), randomEngine);
    }

    for (int s = 0; s < segments; s++) {
        int repetions = 4;
        for (int i = 0; i < repetions; i++) {
            int segmentY = s * segheight;
            if (segmentY > chunk.top) {
                break;
            }
            size_t index = randomPattern[(s * repetions + i + randomTickId) % randomPattern.size()];
            int bx = index % CHUNK_W;
            int bz = (index / CHUNK_W) % CHUNK_D;
            int by = (index / (CHUNK_W * CHUNK_D)) + segmentY;
            const voxel& vox = chunk.voxels[index + segmentY * CHUNK_W * CHUNK_D];
            auto& block = indices->blocks.require(vox.id);
            if (block.rt.funcsset.randupdate) {
                scripting::random_update_block(
                    block,
                    glm::ivec3(
                        chunk.x * CHUNK_W + bx, by, chunk.z * CHUNK_D + bz
                    )
                );
            }
        }
    }
}

void BlocksController::randomTick(int tickid, int parts, uint padding) {
    auto indices = level.content.getIndices();

    for (const auto& [pid, player] : *level.players) {
        const auto& chunks = *player->chunks;
        int width = chunks.getWidth();
        int height = chunks.getHeight();

        for (uint z = padding; z < height - padding; z++) {
            for (uint x = padding; x < width - padding; x++) {
                int index = z * width + x;
                if ((index + tickid) % parts != 0) {
                    continue;
                }
                auto& chunk = chunks.getChunks()[index];
                if (chunk == nullptr || !chunk->flags.ready) {
                    continue;
                }
                if (chunk->lastRandomTickId == randomTickId) {
                    continue;
                }
                chunk->lastRandomTickId = randomTickId;
                randomTick(*chunk, indices);
            }
        }
    }
    randomTickId++;
}

int64_t BlocksController::createBlockInventory(int x, int y, int z) {
    auto chunk = blocks_agent::get_chunk(
        chunks, floordiv<CHUNK_W>(x), floordiv<CHUNK_D>(z)
    );
    if (chunk == nullptr || y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int lx = x - chunk->x * CHUNK_W;
    int lz = z - chunk->z * CHUNK_D;
    auto inv = chunk->getBlockInventory(lx, y, lz);
    if (inv == nullptr) {
        const auto& indices = level.content.getIndices()->blocks;
        auto& def = indices.require(chunk->voxels[vox_index(lx, y, lz)].id);
        int invsize = def.inventorySize;
        if (invsize == 0) {
            return 0;
        }
        inv = level.inventories->create(invsize);
        chunk->addBlockInventory(inv, lx, y, lz);
    }
    return inv->getId();
}

void BlocksController::bindInventory(int64_t invid, int x, int y, int z) {
    auto chunk = blocks_agent::get_chunk(
        chunks, floordiv<CHUNK_W>(x), floordiv<CHUNK_D>(z)
    );
    if (chunk == nullptr) {
        throw std::runtime_error("block does not exists");
    }
    if (invid <= 0) {
        throw std::runtime_error("unable to bind virtual inventory");
    }
    int lx = x - chunk->x * CHUNK_W;
    int lz = z - chunk->z * CHUNK_D;
    chunk->addBlockInventory(level.inventories->get(invid), lx, y, lz);
}

void BlocksController::unbindInventory(int x, int y, int z) {
    auto chunk = blocks_agent::get_chunk(
        chunks, floordiv<CHUNK_W>(x), floordiv<CHUNK_D>(z)
    );
    if (chunk == nullptr) {
        throw std::runtime_error("block does not exists");
    }
    int lx = x - chunk->x * CHUNK_W;
    int lz = z - chunk->z * CHUNK_D;
    chunk->removeBlockInventory(lx, y, lz);
}

void BlocksController::onBlockInteraction(
    Player* player, glm::ivec3 pos, const Block& def, BlockInteraction type
) {
    for (const auto& callback : blockInteractionCallbacks) {
        callback(player, pos, def, type);
    }
}

void BlocksController::listenBlockInteraction(
    const OnBlockInteraction& callback
) {
    blockInteractionCallbacks.push_back(callback);
}
