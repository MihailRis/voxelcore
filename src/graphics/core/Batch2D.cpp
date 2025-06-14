#include "Batch2D.hpp"
#include "Mesh.hpp"
#include "Texture.hpp"
#include "gl_util.hpp"
#include "maths/UVRegion.hpp"

#include <cmath>


Batch2D::Batch2D(size_t capacity) : capacity(capacity), color(1.0f){
    buffer = std::make_unique<Batch2DVertex[]>(capacity );
    mesh = std::make_unique<Mesh<Batch2DVertex>>(buffer.get(), 0);
    index = 0;

    const ubyte pixels[] = {
        0xFF, 0xFF, 0xFF, 0xFF
    };
    ImageData image(ImageFormat::rgba8888, 1, 1, pixels);
    blank = Texture::from(&image);
    currentTexture = nullptr;
}

Batch2D::~Batch2D(){
}

void Batch2D::setPrimitive(DrawPrimitive primitive) {
    if (primitive == this->primitive) {
        return;
    }
    flush();
    this->primitive = primitive;
}

void Batch2D::begin(){
    currentTexture = nullptr;
    blank->bind();
    region = blank->getUVRegion();
    color = glm::vec4(1.0f);
    primitive = DrawPrimitive::triangle;
}

void Batch2D::vertex(
    float x, float y,
    float u, float v,
    float r, float g, float b, float a
) {
    buffer[index].position = {x, y};
    buffer[index].uv = {u * region.getWidth() + region.u1, v * region.getHeight() + region.v1};
    buffer[index].color = {r, g, b, a};
    index++;
}
void Batch2D::vertex(
    glm::vec2 point,
    glm::vec2 uvpoint,
    float r, float g, float b, float a
) {
    buffer[index].position = point;
    buffer[index].uv = {uvpoint.x * region.getWidth() + region.u1, uvpoint.y * region.getHeight() + region.v1};
    buffer[index].color = {r, g, b, a};
    index++;
}

void Batch2D::texture(const Texture* newTexture){
    if (currentTexture == newTexture) {
        return;
    }
    flush();
    currentTexture = newTexture;
    if (newTexture == nullptr) {
        blank->bind();
        region = blank->getUVRegion();
    } else {
        newTexture->bind();
        region = currentTexture->getUVRegion();
    }
}

void Batch2D::untexture() {
    texture(nullptr);
}

void Batch2D::setRegion(UVRegion region) {
    this->region = region;
}

void Batch2D::point(float x, float y, float r, float g, float b, float a){
    if (index + 6 >= capacity)
        flush();
    setPrimitive(DrawPrimitive::point);
    vertex(x, y, 0, 0, r,g,b,a);
}

void Batch2D::line(float x1, float y1, float x2, float y2, float r, float g, float b, float a){
    if (index + 6 >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::line);
    vertex(x1, y1, 0, 0, r,g,b,a);
    vertex(x2, y2, 1, 1, r,g,b,a);
}

void Batch2D::rect(float x, float y, float w, float h){
    const float r = color.r;
    const float g = color.g;
    const float b = color.b;
    const float a = color.a;
    if (index + 6 >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::triangle);
    vertex(x, y, 0, 0, r,g,b,a);
    vertex(x, y+h, 0, 1, r,g,b,a);
    vertex(x+w, y+h, 1, 1, r,g,b,a);

    vertex(x, y, 0, 0, r,g,b,a);
    vertex(x+w, y+h, 1, 1, r,g,b,a);
    vertex(x+w, y, 1, 0, r,g,b,a);
}

void Batch2D::rect(
    float x, float y,
    float w, float h,
    float ox, float oy,
    float angle,
    UVRegion region,
    bool flippedX,
    bool flippedY,
    glm::vec4 tint
) {
    if (index + 6  >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::triangle);
    float centerX = w*ox;
    float centerY = h*oy;
    float acenterX = w-centerX;
    float acenterY = h-centerY;

    float _x1 = -centerX;
    float _y1 = -centerY;

    float _x2 = -centerX;
    float _y2 = +acenterY;

    float _x3 = +acenterX;
    float _y3 = +acenterY;

    float _x4 = +acenterX;
    float _y4 = -centerY;

    float x1,y1,x2,y2,x3,y3,x4,y4;

    constexpr float epsilon = 1e-6f; // 0.000001
    if (std::fabs(angle) > epsilon) {
        float s = std::sin(angle);
        float c = std::cos(angle);

        x1 = c * _x1 - s * _y1;
        y1 = s * _x1 + c * _y1;

        x2 = c * _x2 - s * _y2;
        y2 = s * _x2 + c * _y2;

        x3 = c * _x3 - s * _y3;
        y3 = s * _x3 + c * _y3;

        x4 = x1 + (x3 - x2);
        y4 = y3 - (y2 - y1);
    } else {
        x1 = _x1;
        y1 = _y1;
        x2 = _x2;
        y2 = _y2;
        x3 = _x3;
        y3 = _y3;
        x4 = _x4;
        y4 = _y4;
    }

    x1 += x; x2 += x; x3 += x; x4 += x;
    y1 += y; y2 += y; y3 += y; y4 += y;

    float u1 = region.u1;
    float v1 = region.v1;
    float u2 = region.u1;
    float v2 = region.v2;
    float u3 = region.u2;
    float v3 = region.v2;
    float u4 = region.u2;
    float v4 = region.v1;

    if (flippedX) {
        float temp = u1;
        u1 = u3;
        u4 = temp;
        u2 = u3;
        u3 = temp;
    }
    if (flippedY) {
        float temp = v1;
        v1 = v2;
        v4 = v2;
        v2 = temp;
        v3 = temp;
    }

    vertex(x1, y1, u1, v1, tint.r, tint.g, tint.b, tint.a);
    vertex(x2, y2, u2, v2, tint.r, tint.g, tint.b, tint.a);
    vertex(x3, y3, u3, v3, tint.r, tint.g, tint.b, tint.a);

    /* Right down triangle */
    vertex(x1, y1, u1, v1, tint.r, tint.g, tint.b, tint.a);
    vertex(x3, y3, u3, v3, tint.r, tint.g, tint.b, tint.a);
    vertex(x4, y4, u4, v4, tint.r, tint.g, tint.b, tint.a);
}

void Batch2D::lineRect(float x, float y, float w, float h) {
    if (index + 8  >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::line);

    vertex(x, y, 0.0f, 0.0f, color.r, color.g, color.b, color.a);
    vertex(x, y+h, 0.0f, 1.0f, color.r, color.g, color.b, color.a);

    vertex(x, y+h, 0.0f, 1.0f, color.r, color.g, color.b, color.a);
    vertex(x+w, y+h, 1.0f, 1.0f, color.r, color.g, color.b, color.a);

    vertex(x+w, y+h, 1.0f, 1.0f, color.r, color.g, color.b, color.a);
    vertex(x+w, y, 1.0f, 0.0f, color.r, color.g, color.b, color.a);

    vertex(x+w, y, 1.0f, 0.0f, color.r, color.g, color.b, color.a);
    vertex(x, y, 0.0f, 0.0f, color.r, color.g, color.b, color.a);
}

void Batch2D::rect(
    float x, float y, float w, float h,
    float u, float v, float tx, float ty,
    float r, float g, float b, float a
){
    if (index + 6 >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::triangle);
    vertex(x, y, u, v+ty, r,g,b,a);
    vertex(x+w, y+h, u+tx, v, r,g,b,a);
    vertex(x, y+h, u, v, r,g,b,a);

    vertex(x, y, u, v+ty, r,g,b,a);
    vertex(x+w, y, u+tx, v+ty, r,g,b,a);
    vertex(x+w, y+h, u+tx, v, r,g,b,a);
}

void Batch2D::parallelogram(
    float x, float y, float w, float h, float skew,
    float u, float v, float tx, float ty,
    float r, float g, float b, float a
){
    if (index + 6 >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::triangle);
    vertex(x-skew*w, y, u, v+ty, r,g,b,a);
    vertex(x+(1+skew)*w, y+h, u+tx, v, r,g,b,a);
    vertex(x+skew*w, y+h, u, v, r,g,b,a);

    vertex(x-skew*w, y, u, v+ty, r,g,b,a);
    vertex(x+w-skew*w, y, u+tx, v+ty, r,g,b,a);
    vertex(x+(1+skew)*w, y+h, u+tx, v, r,g,b,a);
}

void Batch2D::rect(
    float x, float y, float w, float h,
    float r0, float g0, float b0,
    float r1, float g1, float b1,
    float r2, float g2, float b2,
    float r3, float g3, float b3,
    float r4, float g4, float b4, int sh
){
    if (index + 30 >= capacity) {
        flush();
    }
    setPrimitive(DrawPrimitive::triangle);
    glm::vec2 v0(x+h/2,y+h/2);
    glm::vec2 v1(x+w-sh,y);
    glm::vec2 v2(x+sh,y);
    glm::vec2 v3(x,y+sh);
    glm::vec2 v4(x,y+h-sh);
    glm::vec2 v5(x+sh,y+h);
    glm::vec2 v6(x+w-h/2,y+h/2);
    glm::vec2 v7(x+w-sh,y+h);
    glm::vec2 v8(x+w,y+h-sh);
    glm::vec2 v9(x+w,y+sh);

    vertex(v0, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v6, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v1, glm::vec2(0, 0), r1,g1,b1,1.0f);

    vertex(v0, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v1, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v2, glm::vec2(0, 0), r1,g1,b1,1.0f);

    vertex(v0, glm::vec2(0, 0), r0,g0,b0,1.0f);
    vertex(v2, glm::vec2(0, 0), r0,g0,b0,1.0f);
    vertex(v3, glm::vec2(0, 0), r0,g0,b0,1.0f);

    vertex(v0, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v3, glm::vec2(0, 0), r1,g1,b1,1.0f);
    vertex(v4, glm::vec2(0, 0), r1,g1,b1,1.0f);

    vertex(v0, glm::vec2(0, 0), r2,g2,b2,1.0f);
    vertex(v4, glm::vec2(0, 0), r2,g2,b2,1.0f);
    vertex(v5, glm::vec2(0, 0), r2,g2,b2,1.0f);

    vertex(v0, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v5, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v6, glm::vec2(0, 0), r3,g3,b3,1.0f);

    vertex(v6, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v5, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v7, glm::vec2(0, 0), r3,g3,b3,1.0f);

    vertex(v6, glm::vec2(0, 0), r4,g4,b4,1.0f);
    vertex(v7, glm::vec2(0, 0), r4,g4,b4,1.0f);
    vertex(v8, glm::vec2(0, 0), r4,g4,b4,1.0f);

    vertex(v6, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v8, glm::vec2(0, 0), r3,g3,b3,1.0f);
    vertex(v9, glm::vec2(0, 0), r3,g3,b3,1.0f);

    vertex(v6, glm::vec2(0, 0), r2,g2,b2,1.0f);
    vertex(v9, glm::vec2(0, 0), r2,g2,b2,1.0f);
    vertex(v1, glm::vec2(0, 0), r2,g2,b2,1.0f);
}

void Batch2D::sprite(float x, float y, float w, float h, const UVRegion& region, glm::vec4 tint){
    rect(x, y, w, h, region.u1, region.v1, region.u2-region.u1, region.v2-region.v1, tint.r, tint.g, tint.b, tint.a);
}

void Batch2D::sprite(float x, float y, float w, float h, int atlasRes, int index, glm::vec4 tint){
    float scale = 1.0f / (float)atlasRes;
    float u = (index % atlasRes) * scale;
    float v = 1.0f - ((index / atlasRes) * scale) - scale;
    rect(x, y, w, h, u, v, scale, scale, tint.r, tint.g, tint.b, tint.a);
}

void Batch2D::sprite(
    float x,
    float y,
    float w,
    float h,
    float skew,
    int atlasRes,
    int index,
    glm::vec4 tint
) {
    float scale = 1.0f / (float)atlasRes;
    float u = (index % atlasRes) * scale;
    float v = 1.0f - ((index / atlasRes) * scale) - scale;
    parallelogram(x, y, w, h, skew, u, v, scale, scale, tint.r, tint.g, tint.b, tint.a);
}

void Batch2D::flush() {
    if (index == 0)
        return;
    mesh->reload(buffer.get(), index);
    mesh->draw(gl::to_glenum(primitive));
    index = 0;
}

void Batch2D::lineWidth(float width) {
    glLineWidth(width);
}
