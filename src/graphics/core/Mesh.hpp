#pragma once

#include <vector>

#include "MeshData.hpp"


struct MeshStats {
    static int meshesCount;
    static int drawCalls;
};

struct IndexBufferData {
    const uint32_t* indices;
    size_t indicesCount;
};

template <typename VertexStructure>
class Mesh {
    struct IndexBuffer {
        unsigned int ibo;
        size_t indexCount;
    };
    unsigned int vao;
    unsigned int vbo;
    std::vector<IndexBuffer> ibos;
    size_t vertexCount;
public:
    explicit Mesh(const MeshData<VertexStructure>& data);

    Mesh(
        const VertexStructure* vertexBuffer,
        size_t vertices,
        std::vector<IndexBufferData> indices
    );

    Mesh(const VertexStructure* vertexBuffer, size_t vertices)
        : Mesh<VertexStructure>(vertexBuffer, vertices, {}) {};

    ~Mesh();

    /// @brief Update GL vertex and index buffers data without changing VAO
    /// attributes
    /// @param vertexBuffer vertex data buffer
    /// @param vertexCount number of vertices in new buffer
    /// @param indices indices buffer
    void reload(
        const VertexStructure* vertexBuffer,
        size_t vertexCount,
        const std::vector<IndexBufferData>& indices
    );

    void reload(const VertexStructure* vertexBuffer, size_t vertexCount) {
        static const std::vector<IndexBufferData> indices {};
        reload(vertexBuffer, vertexCount, indices);
    }

    /// @brief Draw mesh with specified primitives type
    /// @param iboIndex index of used element buffer
    void draw(unsigned int primitive, int iboIndex = 0) const;

    /// @brief Draw mesh as triangles
    void draw() const;
};

#include "graphics/core/Mesh.inl"
