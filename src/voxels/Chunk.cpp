#include "Chunk.hpp"

#include <utility>

#include "content/ContentReport.hpp"
#include "items/Inventory.hpp"
#include "lighting/Lightmap.hpp"
#include "util/data_io.hpp"
#include "voxel.hpp"

Chunk::Chunk(int xpos, int zpos) : x(xpos), z(zpos) {
    bottom = 0;
    top = CHUNK_H;
}

void Chunk::updateHeights() {
    flags.dirtyHeights = false;
    for (uint i = 0; i < CHUNK_VOL; i++) {
        if (voxels[i].id != 0) {
            bottom = i / (CHUNK_D * CHUNK_W);
            break;
        }
    }
    for (int i = CHUNK_VOL - 1; i >= 0; i--) {
        if (voxels[i].id != 0) {
            top = i / (CHUNK_D * CHUNK_W) + 1;
            break;
        }
    }
}

void Chunk::addBlockInventory(
    std::shared_ptr<Inventory> inventory, uint x, uint y, uint z
) {
    inventories[vox_index(x, y, z)] = std::move(inventory);
    flags.unsaved = true;
}

void Chunk::removeBlockInventory(uint x, uint y, uint z) {
    if (inventories.erase(vox_index(x, y, z))) {
        flags.unsaved = true;
    }
}

void Chunk::setBlockInventories(ChunkInventoriesMap map) {
    inventories = std::move(map);
}

std::shared_ptr<Inventory> Chunk::getBlockInventory(uint x, uint y, uint z)
    const {
    if (x >= CHUNK_W || y >= CHUNK_H || z >= CHUNK_D) return nullptr;
    const auto& found = inventories.find(vox_index(x, y, z));
    if (found == inventories.end()) {
        return nullptr;
    }
    return found->second;
}

std::unique_ptr<Chunk> Chunk::clone() const {
    auto other = std::make_unique<Chunk>(x, z);
    for (uint i = 0; i < CHUNK_VOL; i++) {
        other->voxels[i] = voxels[i];
    }
    other->lightmap.set(&lightmap);
    return other;
}

/**
  Current chunk format:
    - byte-order: little-endian

    ```cpp
    uint16_t voxel_id[CHUNK_VOL];
    uint16_t voxel_states[CHUNK_VOL];
    ```

    Total size: (CHUNK_VOL * 4) bytes
*/
std::unique_ptr<ubyte[]> Chunk::encode() const {
    auto buffer = std::make_unique<ubyte[]>(CHUNK_DATA_LEN);
    auto dst = reinterpret_cast<uint16_t*>(buffer.get());
    for (uint i = 0; i < CHUNK_VOL; i++) {
        dst[i] = dataio::h2le(voxels[i].id);
        dst[CHUNK_VOL + i] = dataio::h2le(blockstate2int(voxels[i].state));
    }
    return buffer;
}

bool Chunk::decode(const ubyte* data) {
    auto src = reinterpret_cast<const uint16_t*>(data);
    for (uint i = 0; i < CHUNK_VOL; i++) {
        voxel& vox = voxels[i];

        vox.id = dataio::le2h(src[i]);
        vox.state = int2blockstate(dataio::le2h(src[CHUNK_VOL + i]));
    }
    return true;
}

void Chunk::convert(ubyte* data, const ContentReport* report) {
    auto buffer = reinterpret_cast<uint16_t*>(data);
    for (uint i = 0; i < CHUNK_VOL; i++) {
        blockid_t id = dataio::le2h(buffer[i]);
        blockid_t replacement = report->blocks.getId(id);
        buffer[i] = dataio::h2le(replacement);
    }
}
