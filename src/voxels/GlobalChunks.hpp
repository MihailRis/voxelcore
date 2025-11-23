#pragma once

#include <memory>
#include <unordered_map>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

#include "voxel.hpp"
#include "delegates.hpp"

class Chunk;
class Level;
struct AABB;
class ContentIndices;

class GlobalChunks {
    static inline uint64_t keyfrom(int32_t x, int32_t z) {
        union {
            int32_t pos[2];
            uint64_t key;
        } ekey;
        ekey.pos[0] = x;
        ekey.pos[1] = z;
        return ekey.key;
    }

    Level& level;
    const ContentIndices& indices;
    std::unordered_map<uint64_t, std::shared_ptr<Chunk>> chunksMap;
    std::unordered_map<glm::ivec2, std::shared_ptr<Chunk>> pinnedChunks;
    std::unordered_map<ptrdiff_t, int> refCounters;

    consumer<Chunk&> onUnload;
public:
    GlobalChunks(Level& level);
    ~GlobalChunks() = default;

    void setOnUnload(consumer<Chunk&> onUnload);

    std::shared_ptr<Chunk> fetch(int x, int z);
    std::shared_ptr<Chunk> create(int x, int z, bool lighting);

    void pinChunk(std::shared_ptr<Chunk> chunk);
    void unpinChunk(int x, int z);

    size_t size() const;

    void incref(Chunk* chunk);
    void decref(Chunk* chunk);

    void erase(int x, int z);

    void save(Chunk* chunk);
    void saveAll();

    void putChunk(std::shared_ptr<Chunk> chunk);

    const AABB* isObstacleAt(float x, float y, float z) const;

    inline Chunk* getChunk(int cx, int cz) const {
        const auto& found = chunksMap.find(keyfrom(cx, cz));
        if (found == chunksMap.end()) {
            return nullptr;
        }
        return found->second.get();
    }

    const ContentIndices& getContentIndices() const {
        return indices;
    }
};
