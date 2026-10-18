#pragma once

#include <algorithm>

#include "gl_util.hpp"

template <typename InstanceStructure>
InstanceBuffer<InstanceStructure>::InstanceBuffer() : vbo(0), capacity(0) {
    static_assert(
        calc_size(InstanceStructure::ATTRIBUTES) == sizeof(InstanceStructure)
    );
    glGenBuffers(1, &vbo);
}

template <typename InstanceStructure>
InstanceBuffer<InstanceStructure>::~InstanceBuffer() {
    glDeleteBuffers(1, &vbo);
}

template <typename InstanceStructure>
void InstanceBuffer<InstanceStructure>::reload(
    const InstanceStructure* instances, size_t count
) {
    size_t size = count * sizeof(InstanceStructure);
    if (size > capacity) {
        capacity = std::max(size, capacity * 2);
    }
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, capacity, nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, size, instances);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

template <typename InstanceStructure>
void InstanceBuffer<InstanceStructure>::bind(int location, size_t first) const {
    const auto& attrs = InstanceStructure::ATTRIBUTES;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    size_t offset = first * sizeof(InstanceStructure);
    for (int i = 0; attrs[i].count; i++) {
        const VertexAttribute& attr = attrs[i];
        glVertexAttribPointer(
            location + i,
            attr.count,
            gl::to_glenum(attr.type),
            attr.normalized,
            sizeof(InstanceStructure),
            (GLvoid*)offset
        );
        glVertexAttribDivisor(location + i, 1);
        glEnableVertexAttribArray(location + i);
        offset += attr.size();
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
