#pragma once

#include <vector>
#include <array>
#include <memory>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "graphics/core/MeshData.hpp"
#include "util/Buffer.hpp"

/// @brief Chunk mesh vertex format
struct ChunkVertex {
    glm::vec3 position;
    glm::vec2 uv;
    std::array<uint8_t, 4> color;
    std::array<uint8_t, 4> normal;

    static constexpr VertexAttribute ATTRIBUTES[] = {
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::UNSIGNED_BYTE, true, 4},
        {VertexAttribute::Type::UNSIGNED_BYTE, true, 4},
        {{}, 0}};
};

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
};

struct ChunkMesh {
    std::unique_ptr<Mesh<ChunkVertex> > mesh;
    SortingMeshData sortingMeshData;
    std::unique_ptr<Mesh<ChunkVertex> > sortedMesh = nullptr;
};
