#include "Chunks.hpp"

#include <math.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "data/StructLayout.hpp"
#include "coders/byte_utils.hpp"
#include "content/Content.hpp"
#include "world/files/WorldFiles.hpp"
#include "lighting/Lightmap.hpp"
#include "maths/aabb.hpp"
#include "maths/rays.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "world/Level.hpp"
#include "world/LevelEvents.hpp"
#include "VoxelsVolume.hpp"
#include "blocks_agent.hpp"

Chunks::Chunks(
    int32_t w,
    int32_t d,
    int32_t ox,
    int32_t oz,
    LevelEvents* events,
    const ContentIndices& indices
)
    : events(events),
      indices(indices),
      areaMap(w, d) {
    areaMap.setCenter(ox - w / 2, oz - d / 2);
    areaMap.setOutCallback([this](int, int, const auto& chunk) {
        this->events->trigger(LevelEventType::CHUNK_HIDDEN, chunk.get());
    });
}

void Chunks::configure(int32_t x, int32_t z, uint32_t radius) {
    uint32_t diameter = radius * 2LL;
    if (getWidth() != diameter) {
        resize(diameter, diameter);
    }
    setCenter(x, z);
}

voxel* Chunks::get(int32_t x, int32_t y, int32_t z) const {
    return blocks_agent::get(*this, x, y, z);
}

voxel& Chunks::require(int32_t x, int32_t y, int32_t z) const {
    return blocks_agent::require(*this, x, y, z);
}

const AABB* Chunks::isObstacleAt(float x, float y, float z) const {
    int ix = std::floor(x);
    int iy = std::floor(y);
    int iz = std::floor(z);
    voxel* v = get(ix, iy, iz);
    if (v == nullptr) {
        if (iy >= CHUNK_H) {
            return nullptr;
        } else {
            static const AABB empty;
            return &empty;
        }
    }
    const auto& def = indices.blocks.require(v->id);
    if (def.obstacle) {
        glm::ivec3 offset {};
        if (v->state.segment) {
            glm::ivec3 point(ix, iy, iz);
            offset = seekOrigin(point, def, v->state) - point;
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

bool Chunks::isSolidBlock(int32_t x, int32_t y, int32_t z) {
    return blocks_agent::is_solid_at(*this, x, y, z);
}

bool Chunks::isReplaceableBlock(int32_t x, int32_t y, int32_t z) {
    return blocks_agent::is_replaceable_at(*this, x, y, z);
}

bool Chunks::isObstacleBlock(int32_t x, int32_t y, int32_t z) {
    voxel* v = get(x, y, z);
    if (v == nullptr) return false;
    return indices.blocks.require(v->id).obstacle;
}

ubyte Chunks::getLight(int32_t x, int32_t y, int32_t z, int channel) const {
    if (y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);

    auto ptr = areaMap.getIf(cx, cz);
    if (ptr == nullptr) {
        return 0;
    }
    Chunk* chunk = ptr->get();
    if (chunk == nullptr) {
        return 0;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    return chunk->lightmap.get(lx, y, lz, channel);
}

light_t Chunks::getLight(int32_t x, int32_t y, int32_t z) const {
    if (y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);

    auto ptr = areaMap.getIf(cx, cz);
    if (ptr == nullptr) {
        return 0;
    }
    Chunk* chunk = ptr->get();
    if (chunk == nullptr) {
        return 0;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    return chunk->lightmap.get(lx, y, lz);
}

Chunk* Chunks::getChunkByVoxel(int32_t x, int32_t y, int32_t z) const {
    if (y < 0 || y >= CHUNK_H) {
        return nullptr;
    }
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    if (auto ptr = areaMap.getIf(cx, cz)) {
        return ptr->get();
    }
    return nullptr;
}

Chunk* Chunks::getChunk(int x, int z) const {
    if (auto ptr = areaMap.getIf(x, z)) {
        return ptr->get();
    }
    return nullptr;
}

glm::ivec3 Chunks::seekOrigin(
    const glm::ivec3& srcpos, const Block& def, blockstate state
) const {
    return blocks_agent::seek_origin(*this, srcpos, def, state);
}

void Chunks::eraseSegments(
    const Block& def, blockstate state, int x, int y, int z
) {
    blocks_agent::erase_segments(*this, def, state, x, y, z);
}

void Chunks::repairSegments(
    const Block& def, blockstate state, int x, int y, int z
) {
    blocks_agent::repair_segments(*this, def, state, x, y, z);
}

bool Chunks::checkReplaceability(
    const Block& def,
    blockstate state,
    const glm::ivec3& origin,
    blockid_t ignore
) {
    return blocks_agent::check_replaceability(*this, def, state, origin, ignore);
}

void Chunks::setRotationExtended(
    const Block& def, blockstate state, const glm::ivec3& origin, uint8_t index
) {
    blocks_agent::set_rotation_extended(*this, def, state, origin, index);
}

void Chunks::setRotation(int32_t x, int32_t y, int32_t z, uint8_t index) {
    return blocks_agent::set_rotation(*this, x, y, z, index);
}

void Chunks::set(
    int32_t x, int32_t y, int32_t z, uint32_t id, blockstate state
) {
    blocks_agent::set(*this, x, y, z, id, state);
}

voxel* Chunks::rayCast(
    const glm::vec3& start,
    const glm::vec3& dir,
    float maxDist,
    glm::vec3& end,
    glm::ivec3& norm,
    glm::ivec3& iend,
    std::set<blockid_t> filter
) const {
    return blocks_agent::raycast(
        *this, start, dir, maxDist, end, norm, iend, std::move(filter)
    );
}

glm::vec3 Chunks::rayCastToObstacle(
    const glm::vec3& start, const glm::vec3& dir, float maxDist
) const {
    const float px = start.x;
    const float py = start.y;
    const float pz = start.z;

    float dx = dir.x;
    float dy = dir.y;
    float dz = dir.z;

    float t = 0.0f;
    int ix = floor(px);
    int iy = floor(py);
    int iz = floor(pz);

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

    while (t <= maxDist) {
        voxel* voxel = get(ix, iy, iz);
        if (voxel) {
            const auto& def = indices.blocks.require(voxel->id);
            if (def.obstacle) {
                if (!def.rt.solid) {
                    const std::vector<AABB>& hitboxes =
                        def.rt.hitboxes[voxel->state.rotation];

                    scalar_t distance;
                    glm::ivec3 norm;
                    Ray ray(start, dir);

                    glm::ivec3 offset {};
                    if (voxel->state.segment) {
                        offset = seekOrigin({ix, iy, iz}, def, voxel->state) -
                                 glm::ivec3(ix, iy, iz);
                    }

                    for (const auto& box : hitboxes) {
                        // norm is dummy now, can be inefficient
                        if (ray.intersectAABB(
                                glm::ivec3(ix, iy, iz) + offset,
                                box,
                                maxDist,
                                norm,
                                distance
                            ) > RayRelation::None) {
                            return start + (dir * glm::vec3(distance));
                        }
                    }
                } else {
                    return glm::vec3(px + t * dx, py + t * dy, pz + t * dz);
                }
            }
        }
        if (txMax < tyMax) {
            if (txMax < tzMax) {
                ix += stepx;
                t = txMax;
                txMax += txDelta;
            } else {
                iz += stepz;
                t = tzMax;
                tzMax += tzDelta;
            }
        } else {
            if (tyMax < tzMax) {
                iy += stepy;
                t = tyMax;
                tyMax += tyDelta;
            } else {
                iz += stepz;
                t = tzMax;
                tzMax += tzDelta;
            }
        }
    }
    return glm::vec3(px + maxDist * dx, py + maxDist * dy, pz + maxDist * dz);
}

void Chunks::setCenter(int32_t x, int32_t z) {
    areaMap.setCenter(floordiv<CHUNK_W>(x), floordiv<CHUNK_D>(z));
}

void Chunks::resize(uint32_t newW, uint32_t newD) {
    areaMap.resize(newW, newD);
}

bool Chunks::putChunk(const std::shared_ptr<Chunk>& chunk) {
    if (areaMap.set(chunk->x, chunk->z, chunk)) {
        if (events) {
            events->trigger(LevelEventType::CHUNK_SHOWN, chunk.get());
        }
        return true;
    }
    return false;
}

// reduce nesting on next modification
// 25.06.2024: not now
// 11.11.2024: not now
void Chunks::getVoxels(VoxelsVolume& volume, bool backlight) const {
    voxel* voxels = volume.getVoxels();
    light_t* lights = volume.getLights();
    int x = volume.getX();
    int y = volume.getY();
    int z = volume.getZ();

    int w = volume.getW();
    int h = volume.getH();
    int d = volume.getD();

    int scx = floordiv<CHUNK_W>(x);
    int scz = floordiv<CHUNK_D>(z);

    int ecx = floordiv<CHUNK_W>(x + w);
    int ecz = floordiv<CHUNK_D>(z + d);

    int cw = ecx - scx + 1;
    int cd = ecz - scz + 1;

    // cw*cd chunks will be scanned
    for (int cz = scz; cz < scz + cd; cz++) {
        for (int cx = scx; cx < scx + cw; cx++) {
            const auto chunk = getChunk(cx, cz);
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
                                const auto block =
                                    indices.blocks.get(voxels[vidx].id);
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

void Chunks::saveAndClear() {
    areaMap.clear();
}

void Chunks::remove(int32_t x, int32_t z) {
    areaMap.remove(x, z);
}
