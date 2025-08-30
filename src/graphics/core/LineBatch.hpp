#pragma once

#include <memory>
#include <stdlib.h>
#include <glm/glm.hpp>

#include "commons.hpp"
#include "MeshData.hpp"

template<typename VertexStructure>
class Mesh;

struct LineVertex {
    glm::vec3 position;
    glm::vec4 color;

    static constexpr VertexAttribute ATTRIBUTES[] {
        {VertexAttribute::Type::FLOAT, false, 3},
        {VertexAttribute::Type::FLOAT, false, 4},
        {{}, 0}};
};

class LineBatch : public Flushable {
    std::unique_ptr<Mesh<LineVertex>> mesh;
    std::unique_ptr<LineVertex[]> buffer;
    size_t index;
    size_t capacity;
public:
    LineBatch(size_t capacity=4096);
    ~LineBatch();

    inline void line(const glm::vec3 a, const glm::vec3 b, const glm::vec4 color) {
        line(a.x, a.y, a.z, b.x, b.y, b.z, color.r, color.g, color.b, color.a); 
    }
    void line(float x1, float y1, float z1, float x2, float y2, float z2,
              float r, float g, float b, float a);
    void box(float x, float y, float z, float w, float h, float d,
             float r, float g, float b, float a);

    inline void box(glm::vec3 xyz, glm::vec3 whd, glm::vec4 rgba) {
        box(xyz.x, xyz.y, xyz.z, whd.x, whd.y, whd.z,
            rgba.r, rgba.g, rgba.b, rgba.a);
    }

    void flush() override;
    void lineWidth(float width);
};
