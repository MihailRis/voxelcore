#include "GlobalChunks.hpp"

#include <algorithm>

#include "content/Content.hpp"
#include "coders/json.hpp"
#include "debug/Logger.hpp"
#include "world/files/WorldFiles.hpp"
#include "items/Inventories.hpp"
#include "lighting/Lightmap.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "voxels/blocks_agent.hpp"
#include "typedefs.hpp"
#include "world/LevelEvents.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "Block.hpp"
#include "Chunk.hpp"

static debug::Logger logger("chunks-storage");

GlobalChunks::GlobalChunks(Level& level)
    : level(level), indices(*level.content.getIndices()) {
    chunksMap.max_load_factor(CHUNKS_MAP_MAX_LOAD_FACTOR);
}

void GlobalChunks::setOnUnload(consumer<Chunk&> onUnload) {
    this->onUnload = std::move(onUnload);
}

std::shared_ptr<Chunk> GlobalChunks::fetch(int x, int z) {
    const auto& found = chunksMap.find(keyfrom(x, z));
    if (found == chunksMap.end()) {
        return nullptr;
    }
    return found->second;
}

static void check_voxels(const ContentIndices& indices, Chunk& chunk) {
    bool corrupted = false;
    blockid_t defsCount = indices.blocks.count();
    for (size_t i = 0; i < CHUNK_VOL; i++) {
        blockid_t id = chunk.voxels[i].id;
        if (id >= defsCount) {
            if (!corrupted) {
#ifdef NDEBUG
                // release
                auto logline = logger.error();
                logline << "corruped blocks detected at " << i << " of chunk ";
                logline << chunk.x << "x" << chunk.z;
                logline << " -> " << id;
                corrupted = true;
#else
                // debug
                abort();
#endif
            }
            chunk.voxels[i].id = BLOCK_AIR;
        }
    }
}

void GlobalChunks::erase(int x, int z) {
    chunksMap.erase(keyfrom(x, z));
}

static inline auto load_inventories(
    WorldRegions& regions,
    const Chunk& chunk,
    const ContentUnitIndices<Block>& defs
) {
    auto invs = regions.fetchInventories(chunk.x, chunk.z);
    auto iterator = invs.begin();
    while (iterator != invs.end()) {
        uint index = iterator->first;
        const auto& def = defs.require(chunk.voxels[index].id);
        if (def.inventorySize == 0) {
            iterator = invs.erase(iterator);
            continue;
        }
        auto& inventory = iterator->second;
        if (def.inventorySize != inventory->size()) {
            inventory->resize(def.inventorySize);
        }
        ++iterator;
    }
    return invs;
}

std::shared_ptr<Chunk> GlobalChunks::create(int x, int z) {
    const auto& found = chunksMap.find(keyfrom(x, z));
    if (found != chunksMap.end()) {
        return found->second;
    }

    auto chunk = std::make_shared<Chunk>(x, z);
    chunksMap[keyfrom(x, z)] = chunk;

    World& world = *level.getWorld();
    auto& regions = world.wfile.get()->getRegions();

    if (auto data = regions.getVoxels(chunk->x, chunk->z)) {
        const auto& indices = *level.content.getIndices();

        chunk->decode(data.get());
        check_voxels(indices, *chunk);

        chunk->setBlockInventories(
            load_inventories(regions, *chunk, indices.blocks)
        );

        auto entitiesData = regions.fetchEntities(chunk->x, chunk->z);
        if (entitiesData.getType() == dv::value_type::object) {
            level.entities->loadEntities(std::move(entitiesData));
            chunk->flags.entities = true;
        }

        chunk->flags.loaded = true;
        for (auto& entry : chunk->inventories) {
            level.inventories->store(entry.second);
        }
    }
    if (auto lights = regions.getLights(chunk->x, chunk->z)) {
        chunk->lightmap.set(lights.get());
        chunk->flags.loadedLights = true;
    }
    chunk->blocksMetadata = regions.getBlocksData(chunk->x, chunk->z);

    level.events->trigger(LevelEventType::CHUNK_PRESENT, chunk.get());
    return chunk;
}

void GlobalChunks::pinChunk(std::shared_ptr<Chunk> chunk) {
    pinnedChunks[{chunk->x, chunk->z}] = std::move(chunk);
}

void GlobalChunks::unpinChunk(int x, int z) {
    pinnedChunks.erase({x, z});
}

size_t GlobalChunks::size() const {
    return chunksMap.size();
}

void GlobalChunks::incref(Chunk* chunk) {
    auto key = reinterpret_cast<ptrdiff_t>(chunk);
    const auto& found = refCounters.find(key);
    if (found == refCounters.end()) {
        refCounters[key] = 1;
        return;
    }
    found->second++;
}

void GlobalChunks::decref(Chunk* chunk) {
    auto key = reinterpret_cast<ptrdiff_t>(chunk);
    const auto& found = refCounters.find(key);
    if (found == refCounters.end()) {
        abort();
    }
    if (--found->second == 0) {
        union {
            int pos[2];
            long long key;
        } ekey;
        ekey.pos[0] = chunk->x;
        ekey.pos[1] = chunk->z;

        save(chunk);
        if (onUnload) {
            onUnload(*chunk);
        }
        chunksMap.erase(ekey.key);
        refCounters.erase(found);
    }
}

void GlobalChunks::save(Chunk* chunk) {
    if (chunk == nullptr) {
        return;
    }
    AABB aabb = chunk->getAABB();
    auto entities = level.entities->getAllInside(aabb);
    auto root = dv::object();
    root["data"] = level.entities->serialize(entities);
    if (!entities.empty()) {
        chunk->flags.entities = true;
    }
    level.getWorld()->wfile->getRegions().put(
        chunk,
        chunk->flags.entities ? json::to_binary(root, true)
                                : std::vector<ubyte>()
    );
}

void GlobalChunks::saveAll() {
    for (const auto& [_, chunk] : chunksMap) {
        save(chunk.get());
    }
}

void GlobalChunks::putChunk(std::shared_ptr<Chunk> chunk) {
    chunksMap[keyfrom(chunk->x, chunk->z)] = std::move(chunk);
}

const AABB* GlobalChunks::isObstacleAt(float x, float y, float z) const {
    return blocks_agent::is_obstacle_at(*this, x, y, z);
}
