#include "blocks_agent.hpp"

#include "maths/rays.hpp"

#include <limits>

using namespace blocks_agent;

template <class Storage>
static inline bool set_block(
    Storage& chunks,
    int32_t x,
    int32_t y,
    int32_t z,
    uint32_t id,
    blockstate state
) {
    if (y < 0 || y >= CHUNK_H) {
        return false;
    }
    const auto& indices = chunks.getContentIndices();
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    Chunk* chunk = get_chunk(chunks, cx, cz);
    if (chunk == nullptr) {
        return false;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    size_t index = vox_index(lx, y, lz);

    // block finalization
    voxel& vox = chunk->voxels[(y * CHUNK_D + lz) * CHUNK_W + lx];
    const auto& prevdef = indices.blocks.require(vox.id);
    if (prevdef.inventorySize != 0) {
        chunk->removeBlockInventory(lx, y, lz);
    }
    if (prevdef.rt.extended && !vox.state.segment) {
        erase_segments(chunks, prevdef, vox.state, x, y, z);
    }
    if (prevdef.dataStruct) {
        if (auto found = chunk->blocksMetadata.find(index)) {
            chunk->blocksMetadata.free(found);
            chunk->flags.unsaved = true;
            chunk->flags.blocksData = true;
        }
    }

    // block initialization
    const auto& newdef = indices.blocks.require(id);
    vox.id = id;
    vox.state = state;
    chunk->setModifiedAndUnsaved();
    if (!state.segment && newdef.rt.extended) {
        repair_segments(chunks, newdef, state, x, y, z);
    }

    if (y < chunk->bottom)
        chunk->bottom = y;
    else if (y + 1 > chunk->top)
        chunk->top = y + 1;
    else if (id == 0)
        chunk->flags.dirtyHeights = true;


    if (lx == 0 && (chunk = get_chunk(chunks, cx - 1, cz))) {
        chunk->flags.modified = true;
    }
    if (lz == 0 && (chunk = get_chunk(chunks, cx, cz - 1))) {
        chunk->flags.modified = true;
    }
    if (lx == CHUNK_W - 1 && (chunk = get_chunk(chunks, cx + 1, cz))) {
        chunk->flags.modified = true;
    }
    if (lz == CHUNK_D - 1 && (chunk = get_chunk(chunks, cx, cz + 1))) {
        chunk->flags.modified = true;
    }
    return true;
}

bool blocks_agent::set(
    Chunks& chunks,
    int32_t x,
    int32_t y,
    int32_t z,
    uint32_t id,
    blockstate state
) {
    return set_block(chunks, x, y, z, id, state);
}

bool blocks_agent::set(
    GlobalChunks& chunks,
    int32_t x,
    int32_t y,
    int32_t z,
    uint32_t id,
    blockstate state
) {
    return set_block(chunks, x, y, z, id, state);
}

template <class Storage>
static inline voxel* raycast_blocks(
    const Storage& chunks,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
) {
    const auto& blocks = chunks.getContentIndices().blocks;
    float px = start.x;
    float py = start.y;
    float pz = start.z;

    float dx = dir.x;
    float dy = dir.y;
    float dz = dir.z;

    float t = 0.0f;
    int ix = std::floor(px);
    int iy = std::floor(py);
    int iz = std::floor(pz);

    int stepx = (dx > 0.0f) ? 1 : -1;
    int stepy = (dy > 0.0f) ? 1 : -1;
    int stepz = (dz > 0.0f) ? 1 : -1;

    constexpr float infinity = std::numeric_limits<float>::infinity();
    constexpr float epsilon = 1e-6f;  // 0.000001
    float txDelta = (std::fabs(dx) < epsilon) ? infinity : std::fabs(1.0f / dx);
    float tyDelta = (std::fabs(dy) < epsilon) ? infinity : std::fabs(1.0f / dy);
    float tzDelta = (std::fabs(dz) < epsilon) ? infinity : std::fabs(1.0f / dz);

    float xdist = (stepx > 0) ? (ix + 1 - px) : (px - ix);
    float ydist = (stepy > 0) ? (iy + 1 - py) : (py - iy);
    float zdist = (stepz > 0) ? (iz + 1 - pz) : (pz - iz);

    float txMax = (txDelta < infinity) ? txDelta * xdist : infinity;
    float tyMax = (tyDelta < infinity) ? tyDelta * ydist : infinity;
    float tzMax = (tzDelta < infinity) ? tzDelta * zdist : infinity;

    int steppedIndex = -1;

    while (t <= maxDist) {
        voxel* voxel = get(chunks, ix, iy, iz);
        if (voxel == nullptr) {
            return nullptr;
        }

        const auto& def = blocks.require(voxel->id);
        if ((filter.empty() && def.selectable) ||
            (!filter.empty() && filter.find(def.rt.id) == filter.end())) {
            end.x = px + t * dx;
            end.y = py + t * dy;
            end.z = pz + t * dz;
            iend.x = ix;
            iend.y = iy;
            iend.z = iz;

            if (!def.rt.solid) {
                const std::vector<AABB>& hitboxes =
                    def.rotatable ? def.rt.hitboxes[voxel->state.rotation]
                                  : def.hitboxes;

                scalar_t distance = maxDist;
                Ray ray(start, dir);

                bool hit = false;

                glm::vec3 offset {};
                if (voxel->state.segment) {
                    offset = seek_origin(chunks, iend, def, voxel->state) - iend;
                }

                for (auto box : hitboxes) {
                    box.a += offset;
                    box.b += offset;
                    scalar_t boxDistance;
                    glm::ivec3 boxNorm;
                    if (ray.intersectAABB(
                            iend, box, maxDist, boxNorm, boxDistance
                        ) > RayRelation::None &&
                        boxDistance < distance) {
                        hit = true;
                        distance = boxDistance;
                        norm = boxNorm;
                        end = start + (dir * glm::vec3(distance));
                    }
                }

                if (hit) return voxel;
            } else {
                iend.x = ix;
                iend.y = iy;
                iend.z = iz;

                norm.x = norm.y = norm.z = 0;
                if (steppedIndex == 0) norm.x = -stepx;
                if (steppedIndex == 1) norm.y = -stepy;
                if (steppedIndex == 2) norm.z = -stepz;
                return voxel;
            }
        }
        if (txMax < tyMax) {
            if (txMax < tzMax) {
                ix += stepx;
                t = txMax;
                txMax += txDelta;
                steppedIndex = 0;
            } else {
                iz += stepz;
                t = tzMax;
                tzMax += tzDelta;
                steppedIndex = 2;
            }
        } else {
            if (tyMax < tzMax) {
                iy += stepy;
                t = tyMax;
                tyMax += tyDelta;
                steppedIndex = 1;
            } else {
                iz += stepz;
                t = tzMax;
                tzMax += tzDelta;
                steppedIndex = 2;
            }
        }
    }
    iend.x = ix;
    iend.y = iy;
    iend.z = iz;

    end.x = px + t * dx;
    end.y = py + t * dy;
    end.z = pz + t * dz;
    norm.x = norm.y = norm.z = 0;
    return nullptr;
}

voxel* blocks_agent::raycast(
    const Chunks& chunks,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
) {
    return raycast_blocks(chunks, start, dir, maxDist, end, norm, iend, filter);
}

voxel* blocks_agent::raycast(
    const GlobalChunks& chunks,
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
) {
    return raycast_blocks(chunks, start, dir, maxDist, end, norm, iend, filter);
}

// reduce nesting on next modification
// 25.06.2024: not now
// 11.11.2024: not now
template <class Storage>
inline void get_voxels_impl(
    const Storage& chunks, VoxelsVolume* volume, bool backlight
) {
    const auto& blocks = chunks.getContentIndices().blocks;
    voxel* voxels = volume->getVoxels();
    light_t* lights = volume->getLights();
    int x = volume->getX();
    int y = volume->getY();
    int z = volume->getZ();

    int w = volume->getW();
    int h = volume->getH();
    int d = volume->getD();

    int scx = floordiv<CHUNK_W>(x);
    int scz = floordiv<CHUNK_D>(z);

    int ecx = floordiv<CHUNK_W>(x + w);
    int ecz = floordiv<CHUNK_D>(z + d);

    int cw = ecx - scx + 1;
    int cd = ecz - scz + 1;

    // cw*cd chunks will be scanned
    for (int cz = scz; cz < scz + cd; cz++) {
        for (int cx = scx; cx < scx + cw; cx++) {
            const auto chunk = get_chunk(chunks, cx, cz);
            if (chunk == nullptr) {
                // no chunk loaded -> filling with BLOCK_VOID
                for (int ly = y; ly < y + h; ly++) {
                    for (int lz = std::max(z, cz * CHUNK_D);
                             lz < std::min(z + d, (cz + 1) * CHUNK_D);
                             lz++) {
                        for (int lx = std::max(x, cx * CHUNK_W);
                                 lx < std::min(x + w, (cx + 1) * CHUNK_W);
                                 lx++) {
                            uint idx = vox_index(lx - x, ly - y, lz - z, w, d);
                            voxels[idx].id = BLOCK_VOID;
                            lights[idx] = 0;
                        }
                    }
                }
            } else {
                const voxel* cvoxels = chunk->voxels;
                const light_t* clights = chunk->lightmap.getLights();
                for (int ly = y; ly < y + h; ly++) {
                    for (int lz = std::max(z, cz * CHUNK_D);
                             lz < std::min(z + d, (cz + 1) * CHUNK_D);
                             lz++) {
                        for (int lx = std::max(x, cx * CHUNK_W);
                                 lx < std::min(x + w, (cx + 1) * CHUNK_W);
                                 lx++) {
                            uint vidx = vox_index(lx - x, ly - y, lz - z, w, d);
                            uint cidx = vox_index(
                                lx - cx * CHUNK_W,
                                ly,
                                lz - cz * CHUNK_D,
                                CHUNK_W,
                                CHUNK_D
                            );
                            voxels[vidx] = cvoxels[cidx];
                            light_t light = clights[cidx];
                            if (backlight) {
                                const auto block = blocks.get(voxels[vidx].id);
                                if (block && block->lightPassing) {
                                    light = Lightmap::combine(
                                        std::min(15,
                                            Lightmap::extract(light, 0) + 1),
                                        std::min(15,
                                            Lightmap::extract(light, 1) + 1),
                                        std::min(15,
                                            Lightmap::extract(light, 2) + 1),
                                        std::min(15, 
                                            static_cast<int>(Lightmap::extract(light, 3)))
                                    );
                                }
                            }
                            lights[vidx] = light;
                        }
                    }
                }
            }
        }
    }
}

void blocks_agent::get_voxels(
    const Chunks& chunks, VoxelsVolume* volume, bool backlight
) {
    get_voxels_impl(chunks, volume, backlight);
}

void blocks_agent::get_voxels(
    const GlobalChunks& chunks, VoxelsVolume* volume, bool backlight
) {
    get_voxels_impl(chunks, volume, backlight);
}
