#pragma once

#include "typedefs.hpp"
#include "commons.hpp"
#include "MeshData.hpp"
#include "maths/UVRegion.hpp"

#include <memory>
#include <cstdlib>
#include <glm/glm.hpp>

template<typename VertexStructure> class Mesh;

class Texture;
struct UVRegion;

struct Batch3DVertex {
    glm::vec3 position;
    glm::vec2 uv;
    glm::vec4 color;

    static constexpr VertexAttribute ATTRIBUTES[] {
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::FLOAT, false, 4},
        {{}, 0}};
};

class Batch3D : public Flushable {
    std::unique_ptr<Batch3DVertex[]> buffer;
    size_t capacity;
    std::unique_ptr<Mesh<Batch3DVertex>> mesh;
    std::unique_ptr<Texture> blank;
    size_t index;
    glm::vec4 tint {1.0f};
    const Texture* currentTexture;
    UVRegion region {0.0f, 0.0f, 1.0f, 1.0f};

    void vertex(
        float x, float y, float z,
        float u, float v,
        float r, float g, float b, float a
    );
    void vertex(
        glm::vec3 coord,
        float u, float v,
        float r, float g, float b, float a
    );
    void vertex(
        glm::vec3 point, glm::vec2 uvpoint,
        float r, float g, float b, float a
    );
    void face(
        const glm::vec3& coord, float w, float h,
        const glm::vec3& axisX,
        const glm::vec3& axisY,
        const UVRegion& region,
        const glm::vec4& tint
    );

public:
    Batch3D(size_t capacity);
    ~Batch3D();

    void begin();
    void texture(const Texture* texture);
    void sprite(
        const glm::vec3& pos,
        const glm::vec3& up,
        const glm::vec3& right,
        float w,
        float h,
        const UVRegion& uv,
        const glm::vec4& tint
    );
    void sprite(
        const glm::vec3& pos,
        const glm::vec3& up,
        const glm::vec3& right,
        float w,
        float h,
        int atlasRes,
        int index,
        const glm::vec4& tint
    );
    void xSprite(
        float w,
        float h,
        const UVRegion& uv,
        const glm::vec4& tint,
        bool shading = true
    );
    void cube(
        const glm::vec3& coords,
        const glm::vec3& size,
        const UVRegion (&texfaces)[6],
        const glm::vec4& tint,
        bool shading = true
    );
    void blockCube(
        const glm::vec3& size,
        const UVRegion (&texfaces)[6],
        const glm::vec4& tint,
        bool shading = true
    );
    void setRegion(UVRegion region);
    void vertex(const glm::vec3& pos, const glm::vec2& uv, const glm::vec3& norm);
    void vertex(const glm::vec3& pos, const glm::vec2& uv, const glm::vec4& tint);
    void point(const glm::vec3& pos, const glm::vec4& tint);
    void flush() override;
    void flushPoints();

    void setColor(const glm::vec4& color);
    const glm::vec4& getColor() const;
};
