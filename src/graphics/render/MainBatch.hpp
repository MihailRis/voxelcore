#pragma once

#include <array>
#include <memory>
#include <cstdint>
#include <glm/glm.hpp>

#include "typedefs.hpp"
#include "maths/UVRegion.hpp"
#include "graphics/core/MeshData.hpp"

template<typename VertexStructure>
class Mesh;
class Texture;
class Chunks;

struct MainBatchVertex {
    glm::vec3 position;
    glm::vec2 uv;
    glm::vec3 tint;
    std::array<uint8_t,4> color;

    static constexpr VertexAttribute ATTRIBUTES[] = {
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::UNSIGNED_BYTE, true, 4},
        {{}, 0}};
};

class MainBatch {
    std::unique_ptr<MainBatchVertex[]> const buffer;
    size_t const capacity;
    size_t index;

    UVRegion region {0.0f, 0.0f, 1.0f, 1.0f};

    std::unique_ptr<Mesh<MainBatchVertex>> mesh;
    std::unique_ptr<Texture> blank;

    const Texture* texture = nullptr;
public:
    /// xyz, uv, color, compressed lights

    MainBatch(size_t capacity);

    ~MainBatch();

    void begin();

    void prepare(int vertices);
    void setTexture(const Texture* texture);
    void setTexture(const Texture* texture, const UVRegion& region);
    void flush();

    static glm::vec4 sampleLight(
        const glm::vec3& pos, const Chunks& chunks, bool backlight
    );

    inline void vertex(
        const glm::vec3& pos,
        const glm::vec2& uv,
        const glm::vec4& light,
        const glm::vec3& tint
    ) {
        MainBatchVertex* buffer = this->buffer.get();
        buffer[index].position = pos;
        buffer[index].uv = {uv.x * region.getWidth() + region.u1,uv.y * region.getHeight() + region.v1};
        buffer[index].tint = tint;

        buffer[index].color[0] = static_cast<uint8_t>(light.r * 255);
        buffer[index].color[1] = static_cast<uint8_t>(light.g * 255);
        buffer[index].color[2] = static_cast<uint8_t>(light.b * 255);
        buffer[index].color[3] = static_cast<uint8_t>(light.a * 255);
        index++;
    }

    inline void quad(
        const glm::vec3& pos,
        const glm::vec3& right,
        const glm::vec3& up,
        const glm::vec2& size,
        const glm::vec4& light,
        const glm::vec3& tint,
        const UVRegion& subregion
    ) {
        prepare(6);
        vertex(
            pos - right * size.x * 0.5f - up * size.y * 0.5f,
            {subregion.u1, subregion.v1},
            light,
            tint
        );
        vertex(
            pos + right * size.x * 0.5f - up * size.y * 0.5f,
            {subregion.u2, subregion.v1},
            light,
            tint
        );
        vertex(
            pos + right * size.x * 0.5f + up * size.y * 0.5f,
            {subregion.u2, subregion.v2},
            light,
            tint
        );

        vertex(
            pos - right * size.x * 0.5f - up * size.y * 0.5f,
            {subregion.u1, subregion.v1},
            light,
            tint
        );
        vertex(
            pos + right * size.x * 0.5f + up * size.y * 0.5f,
            {subregion.u2, subregion.v2},
            light,
            tint
        );
        vertex(
            pos - right * size.x * 0.5f + up * size.y * 0.5f,
            {subregion.u1, subregion.v2},
            light,
            tint
        );
    }

    void cube(
        const glm::vec3& coord,
        const glm::vec3& size,
        const UVRegion(&texfaces)[6],
        const glm::vec4& tint,
        bool shading
    );
};
