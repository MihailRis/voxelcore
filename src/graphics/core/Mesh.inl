#pragma once

#include "MeshData.hpp"
#include "gl_util.hpp"

inline constexpr size_t calc_size(const VertexAttribute attrs[]) {
    size_t vertexSize = 0;
    for (int i = 0; attrs[i].count; i++) {
        vertexSize += attrs[i].size();
    }
    return vertexSize;
}

template <typename VertexStructure>
inline std::vector<IndexBufferData> convert_to_ibd(const MeshData<VertexStructure>& data) {
    std::vector<IndexBufferData> indices;
    for (const auto& buffer : data.indices) {
        indices.push_back(IndexBufferData {buffer.data(), buffer.size()});
    }
    return indices;
}

template <typename VertexStructure>
Mesh<VertexStructure>::Mesh(const MeshData<VertexStructure>& data)
    : Mesh(
          data.vertices.data(),
          data.vertices.size(),
          convert_to_ibd<VertexStructure>(data)
      ) {
}

template <typename VertexStructure>
Mesh<VertexStructure>::Mesh(
    const VertexStructure* vertexBuffer,
    size_t vertices,
    std::vector<IndexBufferData> indices
)
    : vao(0), vbo(0), ibos(), vertexCount(0) {
    static_assert(
        calc_size(VertexStructure::ATTRIBUTES) == sizeof(VertexStructure)
    );
    
    const auto& attrs = VertexStructure::ATTRIBUTES;
    MeshStats::meshesCount++;

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    reload(vertexBuffer, vertices, std::move(indices));

    glBindVertexArray(vao);
    // attributes
    int offset = 0;
    for (int i = 0; attrs[i].count; i++) {
        const VertexAttribute& attr = attrs[i];
        glVertexAttribPointer(
            i,
            attr.count,
            gl::to_glenum(attr.type),
            attr.normalized,
            sizeof(VertexStructure),
            (GLvoid*)(size_t)offset
        );
        glEnableVertexAttribArray(i);
        offset += attr.size();
    }

    glBindVertexArray(0);
}

template <typename VertexStructure>
Mesh<VertexStructure>::~Mesh() {
    MeshStats::meshesCount--;
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    for (int i = ibos.size() - 1; i >= 0; i--) {
        glDeleteBuffers(1, &ibos[i].ibo);
    }
}

template <typename VertexStructure>
void Mesh<VertexStructure>::reload(
    const VertexStructure* vertexBuffer,
    size_t vertexCount,
    const std::vector<IndexBufferData>& indices
) {
    this->vertexCount = vertexCount;
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (vertexBuffer != nullptr && vertexCount != 0) {
        glBufferData(
            GL_ARRAY_BUFFER,
            vertexCount * sizeof(VertexStructure),
            vertexBuffer,
            GL_STREAM_DRAW
        );
    } else {
        glBufferData(GL_ARRAY_BUFFER, 0, {}, GL_STREAM_DRAW);
    }

    for (int i = indices.size(); i < ibos.size(); i++) {
        glDeleteBuffers(1, &ibos[i].ibo);
    }
    ibos.clear();

    for (int i = 0; i < indices.size(); i++) {
        const auto& indexBuffer = indices[i];
        ibos.push_back(IndexBuffer {0, 0});
        glGenBuffers(1, &ibos[i].ibo);
        ibos[i].indexCount = indexBuffer.indicesCount;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibos[i].ibo);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            sizeof(uint32_t) * indexBuffer.indicesCount,
            indexBuffer.indices,
            GL_STATIC_DRAW
        );
    }
    glBindVertexArray(0);
}

template <typename VertexStructure>
void Mesh<VertexStructure>::draw(unsigned int primitive, int iboIndex) const {
    MeshStats::drawCalls++;
    
    if (!ibos.empty()) {
        if (iboIndex < ibos.size()) {
            glBindVertexArray(vao);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibos[iboIndex].ibo);
            glDrawElements(
                primitive, ibos.at(iboIndex).indexCount, GL_UNSIGNED_INT, nullptr
            );
            glBindVertexArray(0);
        }
    } else if (vertexCount > 0) {
        glBindVertexArray(vao);
        glDrawArrays(primitive, 0, vertexCount);
        glBindVertexArray(0);
    }
}

template <typename VertexStructure>
void Mesh<VertexStructure>::draw() const {
    draw(GL_TRIANGLES);
}
