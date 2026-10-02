#pragma once

#include "constants.hpp"
#include "graphics/core/MeshData.hpp"
#include "maths/aabb.hpp"
#include "util/Buffer.hpp"

#include <cstddef>
#include <vector>
#include <array>
#include <memory>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

/// @brief ChunkVertex::flags bit: vertex is not affected by lighting
inline constexpr uint8_t VERTEX_EMISSION_BIT = 0x80;
/// @brief ChunkVertex::flags mask: material shader index
inline constexpr uint8_t VERTEX_MATERIAL_MASK = 0x7F;
static_assert(VERTEX_MATERIAL_MASK == MAX_BLOCK_MATERIAL_SHADERS);

/// @brief Chunk mesh vertex format
struct ChunkVertex {
    glm::vec3 position;
    glm::vec2 uv;
    std::array<uint8_t, 4> color;
    std::array<uint8_t, 3> normal;
    /// @brief emission flag and material shader index, fourth component
    /// of the normal attribute (see VERTEX_EMISSION_BIT, VERTEX_MATERIAL_MASK)
    uint8_t flags;

    static constexpr VertexAttribute ATTRIBUTES[] = {
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::UNSIGNED_BYTE, true, 4},
        {VertexAttribute::Type::UNSIGNED_BYTE, true, 4},
        {{}, 0}};
};
// normal and flags are passed as one 4-components vertex attribute
static_assert(sizeof(ChunkVertex) == 28);
static_assert(
    offsetof(ChunkVertex, flags) == offsetof(ChunkVertex, normal) + 3
);

template<typename VertexStructure>
class Mesh;

struct SortingMeshEntry {
    glm::vec3 position;
    util::Buffer<ChunkVertex> vertexData;
    long long distance;

    inline bool operator<(const SortingMeshEntry &o) const noexcept {
        return distance > o.distance;
    }
};

struct SortingMeshData {
    std::vector<SortingMeshEntry> entries;
};

struct ChunkMeshData {
    MeshData<ChunkVertex> mesh;
    SortingMeshData sortingMesh;
    AABB meshAABB;
};

struct ChunkMesh {
    std::unique_ptr<Mesh<ChunkVertex>> mesh;
    SortingMeshData sortingMeshData;
    std::unique_ptr<Mesh<ChunkVertex> > sortedMesh;
    AABB meshAABB;
};

inline constexpr int VOXELS_BUFFER_PADDING = 2;

template<int, int, int> class StaticVoxelsVolume;

using VoxelsRenderVolume = StaticVoxelsVolume<
    CHUNK_W + VOXELS_BUFFER_PADDING * 2,
    CHUNK_H,
    CHUNK_D + VOXELS_BUFFER_PADDING * 2>;
