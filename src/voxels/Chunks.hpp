#pragma once

#include <stdlib.h>

#include <glm/glm.hpp>
#include <memory>
#include <set>
#include <vector>

#include "typedefs.hpp"
#include "voxel.hpp"
#include "util/AreaMap2D.hpp"

class VoxelRenderer;

struct AABB;
class Content;
class ContentIndices;
class Chunk;
class WorldFiles;
class LevelEvents;
class Block;
class VoxelsVolume;

/// Player-centred chunks matrix
class Chunks {
    LevelEvents* events;
    const ContentIndices& indices;

    void eraseSegments(const Block& def, blockstate state, int x, int y, int z);
    void restoreSegments(
        const Block& def, blockstate state, int x, int y, int z
    );
    void setRotationExtended(
        const Block& def,
        blockstate state,
        const glm::ivec3& origin,
        uint8_t rotation
    );

    util::AreaMap2D<std::shared_ptr<Chunk>, int32_t> areaMap;
public:
    Chunks(
        int32_t w,
        int32_t d,
        int32_t ox,
        int32_t oz,
        LevelEvents* events,
        const ContentIndices& indices
    );
    ~Chunks() = default;

    void configure(int32_t x, int32_t z, uint32_t radius);

    bool putChunk(const std::shared_ptr<Chunk>& chunk);

    Chunk* getChunk(int32_t x, int32_t z) const;
    Chunk* getChunkByVoxel(int32_t x, int32_t y, int32_t z) const;

    template <typename T>
    Chunk* getChunkByVoxel(const glm::vec<3, T>& pos) const {
        return getChunkByVoxel(
            glm::floor(pos.x), glm::floor(pos.y), glm::floor(pos.z)
        );
    }

    voxel* get(int32_t x, int32_t y, int32_t z) const;
    voxel& require(int32_t x, int32_t y, int32_t z) const;

    inline voxel* get(const glm::ivec3& pos) {
        return get(pos.x, pos.y, pos.z);
    }

    inline const voxel* get(const glm::ivec3& pos) const {
        return get(pos.x, pos.y, pos.z);
    }

    light_t getLight(int32_t x, int32_t y, int32_t z) const;
    ubyte getLight(int32_t x, int32_t y, int32_t z, int channel) const;
    void set(int32_t x, int32_t y, int32_t z, uint32_t id, blockstate state);

    /// @brief Seek for the extended block origin position
    /// @param pos segment block position
    /// @param def segment block definition
    /// @param state segment block state
    /// @return origin block position or `pos` if block is not extended
    glm::ivec3 seekOrigin(
        const glm::ivec3& pos, const Block& def, blockstate state
    ) const;

    /// @brief Check if required zone is replaceable
    /// @param def definition of the block that requires a replaceable zone
    /// @param state the block state
    /// @param coord position of the zone start
    /// @param ignore ignored block id (will be counted as replaceable)
    bool checkReplaceability(
        const Block& def,
        blockstate state,
        const glm::ivec3& coord,
        blockid_t ignore = 0
    );

    void setRotation(int32_t x, int32_t y, int32_t z, uint8_t rotation);

    voxel* rayCast(
        const glm::vec3& start,
        const glm::vec3& dir,
        float maxLength,
        glm::vec3& end,
        glm::ivec3& norm,
        glm::ivec3& iend,
        std::set<blockid_t> filter = {}
    ) const;

    glm::vec3 rayCastToObstacle(
        const glm::vec3& start, const glm::vec3& dir, float maxDist
    ) const;

    const AABB* isObstacleAt(float x, float y, float z) const;

    const AABB* isObstacleAt(const glm::vec3& pos) const {
        return isObstacleAt(pos.x, pos.y, pos.z);
    }
    
    bool isSolidBlock(int32_t x, int32_t y, int32_t z);
    bool isReplaceableBlock(int32_t x, int32_t y, int32_t z);
    bool isObstacleBlock(int32_t x, int32_t y, int32_t z);

    void getVoxels(VoxelsVolume& volume, bool backlight = false) const;

    void setCenter(int32_t x, int32_t z);
    void resize(uint32_t newW, uint32_t newD);

    void saveAndClear();

    void remove(int32_t x, int32_t z);

    const std::vector<std::shared_ptr<Chunk>>& getChunks() const {
        return areaMap.getBuffer();
    }

    int getWidth() const {
        return areaMap.getWidth();
    }

    int getHeight() const {
        return areaMap.getHeight();
    }

    int getOffsetX() const {
        return areaMap.getOffsetX();
    }

    int getOffsetY() const {
        return areaMap.getOffsetY();
    }

    size_t getChunksCount() const {
        return areaMap.count();
    }

    size_t getVolume() const {
        return areaMap.area();
    }

    const ContentIndices& getContentIndices() const {
        return indices;
    }

    static inline constexpr unsigned matrixSize(int loadDistance, int padding) {
        return (loadDistance + padding) * 2;
    }
};
