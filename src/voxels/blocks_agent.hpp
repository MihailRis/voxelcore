#pragma once

/// blocks_agent is set of templates but not a class to minimize OOP overhead.

#include "voxel.hpp"
#include "Block.hpp"
#include "Chunk.hpp"
#include "Chunks.hpp"
#include "VoxelsVolume.hpp"
#include "GlobalChunks.hpp"
#include "constants.hpp"
#include "typedefs.hpp"
#include "content/Content.hpp"
#include "maths/voxmaths.hpp"

#include <algorithm>
#include <set>
#include <algorithm>
#include <stdint.h>
#include <stdexcept>
#include <glm/glm.hpp>

struct AABB;

namespace blocks_agent {

/// @brief Get specified chunk.
/// @tparam Storage 
/// @param chunks 
/// @param cx chunk grid position X
/// @param cz chunk grid position Z
/// @return chunk or nullptr if does not exists
template<class Storage>
inline Chunk* get_chunk(const Storage& chunks, int cx, int cz) {
    return chunks.getChunk(cx, cz);
}

/// @brief Get voxel at specified position.
/// Returns nullptr if voxel does not exists. 
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param x position X
/// @param y position Y
/// @param z position Z
/// @return voxel pointer or nullptr
template<class Storage>
inline voxel* get(const Storage& chunks, int32_t x, int32_t y, int32_t z) {
    if (y < 0 || y >= CHUNK_H) {
        return nullptr;
    }
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    Chunk* chunk = get_chunk(chunks, cx, cz);
    if (chunk == nullptr) {
        return nullptr;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    return &chunk->voxels[(y * CHUNK_D + lz) * CHUNK_W + lx];
}

/// @brief Get voxel at specified position.
/// @throws std::runtime_error if voxel does not exists
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param x position X
/// @param y position Y
/// @param z position Z
/// @return voxel reference
template<class Storage>
inline voxel& require(const Storage& chunks, int32_t x, int32_t y, int32_t z) {
    auto vox = get(chunks, x, y, z);
    if (vox == nullptr) {
        throw std::runtime_error("voxel does not exist");
    }
    return *vox;
}

template<class Storage>
inline const Block& get_block_def(const Storage& chunks, blockid_t id) {
    return chunks.getContentIndices().blocks.require(id);
}


/// @brief Check if block at specified position is solid.
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param x position X
/// @param y position Y
/// @param z position Z
/// @return true if block exists and solid
template<class Storage>
inline bool is_solid_at(const Storage& chunks, int32_t x, int32_t y, int32_t z) {
    if (auto vox = get(chunks, x, y, z)) {
        return get_block_def(chunks, vox->id).rt.solid;
    }
    return false;
}

/// @brief Check if block at specified position is replaceable.
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param x position X
/// @param y position Y
/// @param z position Z
/// @return true if block exists and replaceable
template<class Storage>
inline bool is_replaceable_at(const Storage& chunks, int32_t x, int32_t y, int32_t z) {
    if (auto vox = get(chunks, x, y, z)) {
        return get_block_def(chunks, vox->id).replaceable;
    }
    return false;
}

/// @brief Set block at specified position if voxel exists.
/// @param chunks chunks matrix
/// @param x block position X
/// @param y block position Y
/// @param z block position Z
/// @param id new block id
/// @param state new block state
bool set(
    Chunks& chunks,
    int32_t x,
    int32_t y,
    int32_t z,
    uint32_t id,
    blockstate state
);

/// @brief Set block at specified position if voxel exists.
/// @param chunks chunks storage
/// @param x block position X
/// @param y block position Y
/// @param z block position Z
/// @param id new block id
/// @param state new block state
bool set(
    GlobalChunks& chunks,
    int32_t x,
    int32_t y,
    int32_t z,
    uint32_t id,
    blockstate state
);

/// @brief Erase extended block segments
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param def origin block definition
/// @param state origin block state
/// @param x origin position X
/// @param y origin position Y
/// @param z origin position Z
template<class Storage>
inline void erase_segments(
    Storage& chunks, const Block& def, blockstate state, int x, int y, int z
) {
    const auto& rotation = def.rotations.variants[state.rotation];
    for (int sy = 0; sy < def.size.y; sy++) {
        for (int sz = 0; sz < def.size.z; sz++) {
            for (int sx = 0; sx < def.size.x; sx++) {
                if ((sx | sy | sz) == 0) {
                    continue;
                }
                glm::ivec3 pos(x, y, z);
                pos += rotation.axes[0] * sx;
                pos += rotation.axes[1] * sy;
                pos += rotation.axes[2] * sz;
                set(chunks, pos.x, pos.y, pos.z, 0, {});
            }
        }
    }
}

/// @brief Convert segment offset to segment bits
/// @param sx segment offset X
/// @param sy segment offset Y
/// @param sz segment offset Z
/// @return segment bits
static constexpr inline uint8_t segment_to_int(int sx, int sy, int sz) {
    return ((sx > 0) | ((sy > 0) << 1) | ((sz > 0) << 2));
}

/// @brief Restore invalid extended block segments
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param def origin block definition
/// @param state origin block state
/// @param x origin position X
/// @param y origin position Y
/// @param z origin position Z
template <class Storage>
inline void repair_segments(
    Storage& chunks, const Block& def, blockstate state, int x, int y, int z
) {
    const auto& rotation = def.rotations.variants[state.rotation];
    const auto id = def.rt.id;
    const auto size = def.size;
    for (int sy = 0; sy < size.y; sy++) {
        for (int sz = 0; sz < size.z; sz++) {
            for (int sx = 0; sx < size.x; sx++) {
                if ((sx | sy | sz) == 0) {
                    continue;
                }
                blockstate segState = state;
                segState.segment = segment_to_int(sx, sy, sz);

                glm::ivec3 pos(x, y, z);
                pos += rotation.axes[0] * sx;
                pos += rotation.axes[1] * sy;
                pos += rotation.axes[2] * sz;
                set(chunks, pos.x, pos.y, pos.z, id, segState);
            }
        }
    }
}

/// @brief Get origin position for specified extended block. Returns srcpos
/// if block is not extended.
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param srcpos block segment position
/// @param def definition of the block at srcpos
/// @param state state of the block at srcpos
/// @return origin block position
template <class Storage>
inline glm::ivec3 seek_origin(
    Storage& chunks, const glm::ivec3& srcpos, const Block& def, blockstate state
) {
    auto pos = srcpos;
    const auto& rotation = def.rotations.variants[state.rotation];
    auto segment = state.segment;
    while (true) {
        if (!segment) {
            return pos;
        }
        if (segment & 1) pos -= rotation.axes[0];
        if (segment & 2) pos -= rotation.axes[1];
        if (segment & 4) pos -= rotation.axes[2];

        if (auto* voxel = get(chunks, pos.x, pos.y, pos.z)) {
            segment = voxel->state.segment;
        } else {
            return pos;
        }
    }
}

/// @brief Check blocks replaceability with specified block
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param def block definition
/// @param state target block state
/// @param origin target block origin
/// @param ignore ignored block id
/// @return true if specified area may be replaced with the block/extended block
template <class Storage>
inline bool check_replaceability(
    const Storage& chunks,
    const Block& def,
    blockstate state,
    const glm::ivec3& origin,
    blockid_t ignore
) {
    const auto& blocks = chunks.getContentIndices().blocks;
    const auto& rotation = def.rotations.variants[state.rotation];
    const auto size = def.size;
    for (int sy = 0; sy < size.y; sy++) {
        for (int sz = 0; sz < size.z; sz++) {
            for (int sx = 0; sx < size.x; sx++) {
                auto pos = origin;
                pos += rotation.axes[0] * sx;
                pos += rotation.axes[1] * sy;
                pos += rotation.axes[2] * sz;
                if (auto vox = get(chunks, pos.x, pos.y, pos.z)) {
                    auto& target = blocks.require(vox->id);
                    if (!target.replaceable && vox->id != ignore) {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
    }
    return true;
}

/// @brief Set rotation to an extended block
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param def block definition
/// @param state current block state
/// @param origin extended block origin
/// @param index target rotation index
template <class Storage>
inline void set_rotation_extended(
    Storage& chunks,
    const Block& def,
    blockstate state,
    const glm::ivec3& origin,
    uint8_t index
) {
    auto newstate = state;
    newstate.rotation = index;

    // unable to rotate block (cause: obstacles)
    if (!check_replaceability(chunks, def, newstate, origin, def.rt.id)) {
        return;
    }

    const auto& rotation = def.rotations.variants[index];
    const auto size = def.size;
    std::vector<glm::ivec3> segmentBlocks;

    for (int sy = 0; sy < size.y; sy++) {
        for (int sz = 0; sz < size.z; sz++) {
            for (int sx = 0; sx < size.x; sx++) {
                auto pos = origin;
                pos += rotation.axes[0] * sx;
                pos += rotation.axes[1] * sy;
                pos += rotation.axes[2] * sz;

                blockstate segState = newstate;
                segState.segment = segment_to_int(sx, sy, sz);

                auto vox = get(chunks, pos.x, pos.y, pos.z);
                // checked for nullptr by checkReplaceability
                if (vox->id != def.rt.id) {
                    set(chunks, pos.x, pos.y, pos.z, def.rt.id, segState);
                } else {
                    vox->state = segState;
                    int cx = floordiv<CHUNK_W>(pos.x);
                    int cz = floordiv<CHUNK_D>(pos.z);
                    auto chunk = get_chunk(chunks, cx, cz);
                    assert(chunk != nullptr);
                    chunk->setModifiedAndUnsaved();
                    segmentBlocks.emplace_back(pos);
                }
            }
        }
    }
    const auto& prevRotation = def.rotations.variants[state.rotation];
    for (int sy = 0; sy < size.y; sy++) {
        for (int sz = 0; sz < size.z; sz++) {
            for (int sx = 0; sx < size.x; sx++) {
                auto pos = origin;
                pos += prevRotation.axes[0] * sx;
                pos += prevRotation.axes[1] * sy;
                pos += prevRotation.axes[2] * sz;
                if (std::find(
                        segmentBlocks.begin(), segmentBlocks.end(), pos
                    ) == segmentBlocks.end()) {
                    set(chunks, pos.x, pos.y, pos.z, 0, {});
                }
            }
        }
    }
}

/// @brief Set block rotation
/// @tparam Storage chunks storage class
/// @param chunks chunks storage
/// @param x block X position
/// @param y block Y position
/// @param z block Z position
/// @param index target rotation index
template <class Storage>
inline void set_rotation(
    Storage& chunks, int32_t x, int32_t y, int32_t z, uint8_t index
) {
    if (index >= BlockRotProfile::MAX_COUNT) {
        return;
    }
    auto vox = get(chunks, x, y, z);
    if (vox == nullptr) {
        return;
    }
    const auto& def = chunks.getContentIndices().blocks.require(vox->id);
    if (!def.rotatable || vox->state.rotation == index) {
        return;
    }
    if (def.rt.extended) {
        auto origin = seek_origin(chunks, {x, y, z}, def, vox->state);
        vox = get(chunks, origin.x, origin.y, origin.z);
        set_rotation_extended(chunks, def, vox->state, origin, index);
    } else {
        vox->state.rotation = index;
        int cx = floordiv<CHUNK_W>(x);
        int cz = floordiv<CHUNK_D>(z);
        auto chunk = get_chunk(chunks, cx, cz);
        assert(chunk != nullptr);
        chunk->setModifiedAndUnsaved();
    }
}

/// @brief Cast ray to a selectable block with filter based on id.
/// @param chunks chunks matrix
/// @param start ray start position
/// @param dir normalized ray direction vector
/// @param maxDist maximum ray length
/// @param end [out] ray end position
/// @param norm [out] surface normal vector
/// @param iend [out] ray end integer position (voxel position + normal)
/// @param filter filtered ids
/// @return voxel pointer or nullptr
voxel* raycast(
    const Chunks& chunks,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
);

/// @brief Cast ray to a selectable block with filter based on id.
/// @param chunks chunks storage
/// @param start ray start position
/// @param dir normalized ray direction vector
/// @param maxDist maximum ray length
/// @param end [out] ray end position
/// @param norm [out] surface normal vector
/// @param iend [out] ray end integer position (voxel position + normal)
/// @param filter filtered ids
/// @return voxel pointer or nullptr
voxel* raycast(
    const GlobalChunks& chunks,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
);

void get_voxels(const Chunks& chunks, VoxelsVolume* volume, bool backlight=false);

void get_voxels(const GlobalChunks& chunks, VoxelsVolume* volume, bool backlight=false);

template <class Storage>
inline const AABB* is_obstacle_at(const Storage& chunks, float x, float y, float z) {
    int ix = std::floor(x);
    int iy = std::floor(y);
    int iz = std::floor(z);
    voxel* v = get(chunks, ix, iy, iz);
    if (v == nullptr) {
        if (iy >= CHUNK_H) {
            return nullptr;
        } else {
            static const AABB empty;
            return &empty;
        }
    }
    const auto& def = chunks.getContentIndices().blocks.require(v->id);
    if (def.obstacle) {
        glm::ivec3 offset {};
        if (v->state.segment) {
            glm::ivec3 point(ix, iy, iz);
            offset = seek_origin(chunks, point, def, v->state) - point;
        }
        const auto& boxes =
            def.rotatable ? def.rt.hitboxes[v->state.rotation] : def.hitboxes;
        for (const auto& hitbox : boxes) {
            if (hitbox.contains(
                {x - ix - offset.x, y - iy - offset.y, z - iz - offset.z}
            )) {
                return &hitbox;
            }
        }
    }
    return nullptr;
}

} // blocks_agent
