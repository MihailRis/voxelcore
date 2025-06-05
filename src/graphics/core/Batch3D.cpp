#include "Batch3D.hpp"

#include "Mesh.hpp"
#include "Texture.hpp"

#include "typedefs.hpp"
#include "maths/UVRegion.hpp"

namespace {
    const glm::vec3 SUN_VECTOR(0.528265f, 0.833149f, -0.163704f);
    const float DIRECTIONAL_LIGHT_FACTOR = 0.3f;
}

Batch3D::Batch3D(size_t capacity)
    : capacity(capacity) {


    buffer = std::make_unique<Batch3DVertex[]>(capacity);
    mesh = std::make_unique<Mesh<Batch3DVertex>>(buffer.get(), 0);
    index = 0;

    const ubyte pixels[] = {
        255, 255, 255, 255,
    };
    ImageData image(ImageFormat::rgba8888, 1, 1, pixels);
    blank = Texture::from(&image);
    currentTexture = nullptr;
}

Batch3D::~Batch3D(){
}

void Batch3D::begin(){
    currentTexture = nullptr;
    blank->bind();
}

void Batch3D::vertex(
    float x,
    float y,
    float z,
    float u,
    float v,
    float r,
    float g,
    float b,
    float a
) {
    buffer[index].position = {x, y, z};
    buffer[index].uv = {
        u * region.getWidth() + region.u1, v * region.getHeight() + region.v1};
    buffer[index].color = {r, g, b, a};
    index++;
}
void Batch3D::vertex(
    glm::vec3 coord, float u, float v, float r, float g, float b, float a
) {
    buffer[index].position = coord;
    buffer[index].uv = {
        u * region.getWidth() + region.u1, v * region.getHeight() + region.v1};
    buffer[index].color = {r, g, b, a};
    index++;
}
void Batch3D::vertex(
    glm::vec3 point,
    glm::vec2 uvpoint,
    float r, float g, float b, float a
) {
    buffer[index].position = point;
    buffer[index].uv = {
        uvpoint.x * region.getWidth() + region.u1,
        uvpoint.y * region.getHeight() + region.v1};
    buffer[index].color = {r, g, b, a};
    index++;
}

void Batch3D::face(
    const glm::vec3& coord,
    float w, float h,
    const glm::vec3& axisX,
    const glm::vec3& axisY,
    const UVRegion& region,
    const glm::vec4& tint
) {
    if (index + 6 >= capacity) {
        flush();
    }
    vertex(coord, region.u1, region.v1,
           tint.r, tint.g, tint.b, tint.a);
    vertex(coord + axisX * w, region.u2, region.v1,
            tint.r, tint.g, tint.b, tint.a);
    vertex(coord + axisX * w + axisY * h, region.u2, region.v2,
           tint.r, tint.g, tint.b, tint.a);

    vertex(coord, region.u1, region.v1,
           tint.r, tint.g, tint.b, tint.a);
    vertex(coord + axisX * w + axisY * h, region.u2, region.v2,
           tint.r, tint.g, tint.b, tint.a);
    vertex(coord + axisY * h, region.u1, region.v2,
           tint.r, tint.g, tint.b, tint.a);
}

void Batch3D::texture(const Texture* new_texture){
    if (currentTexture == new_texture) {
        return;
    }
    flush();
    currentTexture = new_texture;
    if (new_texture == nullptr) {
        blank->bind();
        region = blank->getUVRegion();
    } else {
        new_texture->bind();
        region = currentTexture->getUVRegion();
    }
}

void Batch3D::sprite(
    const glm::vec3& pos,
    const glm::vec3& up,
    const glm::vec3& right,
    float w,
    float h,
    int atlasRes,
    int index,
    const glm::vec4& tint
) {
    float scale = 1.0f / static_cast<float>(atlasRes);
    float u = (index % atlasRes) * scale;
    float v = 1.0f - ((index / atlasRes) * scale) - scale;
    sprite(pos, up, right, w, h, UVRegion(u, v, u+scale, v+scale), tint);
}

void Batch3D::sprite(
    const glm::vec3& pos,
    const glm::vec3& up,
    const glm::vec3& right,
    float w, float h,
    const UVRegion& uv,
    const glm::vec4& color
){
    const float r = color.r;
    const float g = color.g;
    const float b = color.b;
    const float a = color.a;
    if (index + 6 >= capacity) {
        flush();
    }

    vertex(pos.x - right.x * w - up.x * h,
            pos.y - right.y * w - up.y * h,
            pos.z - right.z * w - up.z * h,
            uv.u1, uv.v1,
            r,g,b,a);

    vertex(pos.x + right.x * w + up.x * h,
            pos.y + right.y * w + up.y * h,
            pos.z + right.z * w + up.z * h,
            uv.u2, uv.v2,
            r,g,b,a);

    vertex(pos.x - right.x * w + up.x * h,
            pos.y - right.y * w + up.y * h,
            pos.z - right.z * w + up.z * h,
            uv.u1, uv.v2,
            r,g,b,a);

    vertex(pos.x - right.x * w - up.x * h,
            pos.y - right.y * w - up.y * h,
            pos.z - right.z * w - up.z * h,
            uv.u1, uv.v1,
            r,g,b,a);

    vertex(pos.x + right.x * w - up.x * h,
            pos.y + right.y * w - up.y * h,
            pos.z + right.z * w - up.z * h,
            uv.u2, uv.v1,
            r,g,b,a);

    vertex(pos.x + right.x * w + up.x * h,
            pos.y + right.y * w + up.y * h,
            pos.z + right.z * w + up.z * h,
            uv.u2, uv.v2,
            r,g,b,a);
}

inline glm::vec4 do_tint(float value) {
    return glm::vec4(value, value, value, 1.0f);
}

void Batch3D::xSprite(
    float w, float h, const UVRegion& uv, const glm::vec4& tint, bool shading
) {
    face(
        glm::vec3(-w * 0.25f, 0.0f, -w * 0.25f),
        w, h,
        glm::vec3(1, 0, 0),
        glm::vec3(0, 1, 0),
        uv, (shading ? do_tint(1.0f)*tint : tint)
    );
    face(
        glm::vec3(w * 0.25f, 0.0f, w * 0.5f - w * 0.25f),
        w, h,
        glm::vec3(0, 0, -1),
        glm::vec3(0, 1,  0),
        uv, (shading ? do_tint(0.9f)*tint : tint)
    );
}

void Batch3D::cube(
    const glm::vec3& coord,
    const glm::vec3& size,
    const UVRegion(&texfaces)[6],
    const glm::vec4& tint,
    bool shading
) {
    const glm::vec3 X(1.0f, 0.0f, 0.0f);
    const glm::vec3 Y(0.0f, 1.0f, 0.0f);
    const glm::vec3 Z(0.0f, 0.0f, 1.0f);

    face(
        coord+glm::vec3(0.0f, 0.0f, 0.0f),
        size.x, size.y, X, Y, texfaces[5],
        (shading ? do_tint(0.8)*tint : tint)
    );
    face(
        coord+glm::vec3(size.x, 0.0f, -size.z),
        size.x, size.y, -X, Y, texfaces[4],
        (shading ? do_tint(0.8f)*tint : tint)
    );
    face(
        coord+glm::vec3(0.0f, size.y, 0.0f),
        size.x, size.z, X, -Z, texfaces[3],
        (shading ? do_tint(1.0f)*tint : tint)
    );
    face(
        coord+glm::vec3(0.0f, 0.0f, -size.z),
        size.x, size.z, X, Z, texfaces[2],
        (shading ? do_tint(0.7f)*tint : tint)
    );
    face(
        coord+glm::vec3(0.0f, 0.0f, -size.z),
        size.z, size.y, Z, Y, texfaces[0],
        (shading ? do_tint(0.9f)*tint : tint)
    );
    face(
        coord+glm::vec3(size.x, 0.0f, 0.0f),
        size.z, size.y, -Z, Y, texfaces[1],
        (shading ? do_tint(0.9f)*tint : tint)
    );
}

void Batch3D::blockCube(
    const glm::vec3& size,
    const UVRegion(&texfaces)[6],
    const glm::vec4& tint,
    bool shading
) {
    cube((1.0f - size) * -0.5f, size, texfaces, tint, shading);
}

void Batch3D::setRegion(UVRegion region) {
    this->region = region;
}

void Batch3D::vertex(const glm::vec3& pos, const glm::vec2& uv, const glm::vec3& norm) {
    float d = glm::dot(glm::normalize(norm), SUN_VECTOR);
    d = (1.0f - DIRECTIONAL_LIGHT_FACTOR) + d * DIRECTIONAL_LIGHT_FACTOR;
    vertex(pos, uv, glm::vec4(d, d, d, 1.0f));
}

void Batch3D::vertex(
    const glm::vec3& coord, const glm::vec2& uv, const glm::vec4& tint
) {
    if (index + 1 >= capacity) {
        flush();
    }
    vertex(coord, uv, tint.r, tint.g, tint.b, tint.a);
}

void Batch3D::point(const glm::vec3& coord, const glm::vec4& tint) {
    if (index + 1 >= capacity) {
        flushPoints();
    }
    vertex(coord, {}, tint.r, tint.g, tint.b, tint.a);
}

void Batch3D::flush() {
    mesh->reload(buffer.get(), index);
    mesh->draw();
    index = 0;
}

void Batch3D::flushPoints() {
    mesh->reload(buffer.get(), index);
    mesh->draw(GL_POINTS);
    index = 0;
}

void Batch3D::setColor(const glm::vec4& color) {
    tint = color;
}

const glm::vec4& Batch3D::getColor() const {
    return tint;
}
