#pragma once

#include <cstddef>

#include "MeshData.hpp"

template <typename InstanceStructure>
class InstanceBuffer {
    unsigned int vbo;
    size_t capacity;
public:
    InstanceBuffer();
    ~InstanceBuffer();

    void reload(const InstanceStructure* instances, size_t count);

    void bind(int location, size_t first) const;
};

#include "graphics/core/InstanceBuffer.inl"
