#pragma once

#include <stdlib.h>

#include <memory>
#include <unordered_map>

#include "constants.hpp"
#include "lighting/Lightmap.hpp"
#include "util/SmallHeap.hpp"
#include "maths/aabb.hpp"
#include "voxel.hpp"

/// @brief Total bytes number of chunk voxel data
inline constexpr int CHUNK_DATA_LEN = CHUNK_VOL * 4;

class ContentReport;
class Inventory;

using ChunkInventoriesMap =
    std::unordered_map<uint, std::shared_ptr<Inventory>>;

using BlocksMetadata = util::SmallHeap<uint16_t, uint8_t>;

class Chunk {
public:
    int x, z;
    int bottom, top;
    voxel voxels[CHUNK_VOL] {};
    Lightmap lightmap;
    struct {
        bool modified : 1;
        bool ready : 1;
        bool loaded : 1;
        bool lighted : 1;
        bool unsaved : 1;
        bool loadedLights : 1;
        bool entities : 1;
        bool blocksData : 1;
        bool dirtyHeights : 1;
    } flags {};

    /// @brief Block inventories map where key is index of block in voxels array
    ChunkInventoriesMap inventories;
    /// @brief Blocks metadata heap
    BlocksMetadata blocksMetadata;

    Chunk(int x, int z);

    /// @brief Refresh `bottom` and `top` values
    void updateHeights();

    // unused
    std::unique_ptr<Chunk> clone() const;

    /// @brief Creates new block inventory given size
    /// @return inventory id or 0 if block does not exists
    void addBlockInventory(
        std::shared_ptr<Inventory> inventory, uint x, uint y, uint z
    );
    void removeBlockInventory(uint x, uint y, uint z);
    void setBlockInventories(ChunkInventoriesMap map);

    /// @return inventory bound to the given block or nullptr
    std::shared_ptr<Inventory> getBlockInventory(uint x, uint y, uint z) const;

    inline void setModifiedAndUnsaved() {
        flags.modified = true;
        flags.unsaved = true;
    }

    /// @brief Encode chunk to bytes array of size CHUNK_DATA_LEN
    /// @see /doc/specs/region_voxels_chunk_spec.md
    std::unique_ptr<ubyte[]> encode() const;

    /// @return true if all is fine
    bool decode(const ubyte* data);

    static void convert(ubyte* data, const ContentReport* report);

    AABB getAABB() const {
        return AABB(
            glm::vec3(x * CHUNK_W, -INFINITY, z * CHUNK_D),
            glm::vec3((x + 1) * CHUNK_W, INFINITY, (z + 1) * CHUNK_D)
        );
    }
};
