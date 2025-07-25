#pragma once

#include <memory>
#include <glm/glm.hpp>

#include "commons.hpp"
#include "maths/UVRegion.hpp"
#include "MeshData.hpp"

template<typename VertexStructure>
class Mesh;
class Texture;

struct Batch2DVertex {
    glm::vec2 position;
    glm::vec2 uv;
    glm::vec4 color;

    static constexpr VertexAttribute ATTRIBUTES[] {
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::FLOAT, false, 2},
        {VertexAttribute::Type::FLOAT, false, 4},
        {{}, 0}};
};

class Batch2D : public Flushable {
    std::unique_ptr<Batch2DVertex[]> buffer;
    size_t capacity;
    std::unique_ptr<Mesh<Batch2DVertex>> mesh;
    std::unique_ptr<Texture> blank;
    size_t index;
    glm::vec4 color;
    const Texture* currentTexture;
    DrawPrimitive primitive = DrawPrimitive::triangle;
    UVRegion region {0.0f, 0.0f, 1.0f, 1.0f};

    void setPrimitive(DrawPrimitive primitive);

    void vertex(
        float x, float y,
        float u, float v,
        float r, float g, float b, float a
    );
    
    void vertex(
        glm::vec2 point,
        glm::vec2 uvpoint,
        float r, float g, float b, float a
    );

public:
    Batch2D(size_t capacity);
    ~Batch2D();

    void begin();
    void texture(const Texture* texture);
    void untexture();
    void setRegion(UVRegion region);
    void sprite(float x, float y, float w, float h, const UVRegion& region, glm::vec4 tint);
    void sprite(float x, float y, float w, float h, int atlasRes, int index, glm::vec4 tint);
    void sprite(float x, float y, float w, float h, float skew, int atlasRes, int index, glm::vec4 tint);
    void point(float x, float y, float r, float g, float b, float a);
    
    void setColor(const glm::vec4& color) {
        this->color = color;
    }

    void setColor(int r, int g, int b, int a=255) {
        this->color = glm::vec4(r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f);
    }

    void resetColor() {
        this->color = glm::vec4(1.0f);
    }

    glm::vec4 getColor() const {
        return color;
    }
    
    void line(
        float x1, float y1, 
        float x2, float y2, 
        float r, float g, float b, float a
    );

    void lineRect(float x, float y, float w, float h);

    void rect(
        float x, float y,
        float w, float h,
        float ox, float oy,
        float angle, UVRegion region,
        bool flippedX, bool flippedY,
        glm::vec4 tint
    );

    void rect(float x, float y, float w, float h);

    void rect(
        float x, float y, float w, float h,
        float u, float v, float tx, float ty,
        float r, float g, float b, float a
    );

    void parallelogram(
        float x, float y, float w, float h, float skew,
        float u, float v, float tx, float ty,
        float r, float g, float b, float a
    );

    void rect(
        float x, float y, float w, float h,
        float r0, float g0, float b0,
        float r1, float g1, float b1,
        float r2, float g2, float b2,
        float r3, float g3, float b3,
        float r4, float g4, float b4, int sh
    );

    void triangle(float x1, float y1, float x2, float y2, float x3, float y3);

    void flush() override;

    void lineWidth(float width);
};
