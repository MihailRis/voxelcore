#pragma once

#include <vector>
#include <stdexcept>

#include "typedefs.hpp"
#include "util/Buffer.hpp"

/// @brief Vertex attribute info
struct VertexAttribute {
    enum class Type {
        FLOAT,
        INT, UNSIGNED_INT,
        SHORT, UNSIGNED_SHORT,
        BYTE, UNSIGNED_BYTE
    };

    Type type = Type::FLOAT;
    bool normalized = false;
    ubyte count = 0;

    [[nodiscard]] constexpr uint32_t size() const {
        switch (type) {
            case Type::FLOAT:
                return count * sizeof(float);
            case Type::UNSIGNED_INT:
            case Type::INT:
                return count * sizeof(int32_t);
            case Type::UNSIGNED_SHORT:
            case Type::SHORT:
                return count * sizeof(int16_t);
            case Type::UNSIGNED_BYTE:
            case Type::BYTE:
                return count * sizeof(int8_t);
        }
        return 0;
    }
};

/// @brief Raw mesh data structure
template<typename VertexStructure>
struct MeshData {
    util::Buffer<VertexStructure> vertices;
    std::vector<util::Buffer<uint32_t>> indices;
    util::Buffer<VertexAttribute> attrs;

    MeshData() = default;

    /// @param vertices vertex data buffer
    /// @param indices nullable indices buffer
    /// @param attrs vertex attribute sizes (must be null-terminated) 
    MeshData(
        util::Buffer<VertexStructure> vertices,
        std::vector<util::Buffer<uint32_t>> indices,
        util::Buffer<VertexAttribute> attrs
    ) : vertices(std::move(vertices)),
        indices(std::move(indices)),
        attrs(std::move(attrs)) {}
};
