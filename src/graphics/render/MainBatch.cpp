#include "MainBatch.hpp"

#include "graphics/core/Texture.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/ImageData.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Chunk.hpp"


MainBatch::MainBatch(size_t capacity)
        : buffer(std::make_unique<MainBatchVertex[]>(capacity)),
          capacity(capacity),
          index(0),
          mesh(std::make_unique<Mesh<MainBatchVertex>>(buffer.get(), 0)) {

    const ubyte pixels[] = {
            255, 255, 255, 255,
    };
    ImageData image(ImageFormat::rgba8888, 1, 1, pixels);
    blank = Texture::from(&image);
}

MainBatch::~MainBatch() = default;

void MainBatch::setTexture(const Texture *texture) {
    if (texture == nullptr) {
        texture = blank.get();
    }
    if (texture != this->texture) {
        flush();
    }
    this->texture = texture;
    region = UVRegion{0.0f, 0.0f, 1.0f, 1.0f};
}

void MainBatch::setTexture(const Texture *texture, const UVRegion &region) {
    setTexture(texture);
    this->region = region;
}

void MainBatch::flush() {
    if (index == 0) {
        return;
    }
    if (texture == nullptr) {
        texture = blank.get();
    }
    texture->bind();
    mesh->reload(buffer.get(), index);
    mesh->draw();
    index = 0;
}

void MainBatch::begin() {
    texture = nullptr;
    blank->bind();
}

void MainBatch::prepare(int vertices) {
    if (index * vertices > capacity) {
        flush();
    }
}

glm::vec4 MainBatch::sampleLight(
        const glm::vec3 &pos, const Chunks &chunks, bool backlight
) {
    light_t light = chunks.getLight(
            std::floor(pos.x),
            std::floor(std::min(CHUNK_H - 1.0f, pos.y)),
            std::floor(pos.z));
    light_t minIntensity = backlight ? 1 : 0;
    return {
            (float) glm::max(Lightmap::extract(light, 0), minIntensity) / 15.0f,
            (float) glm::max(Lightmap::extract(light, 1), minIntensity) / 15.0f,
            (float) glm::max(Lightmap::extract(light, 2), minIntensity) / 15.0f,
            (float) glm::max(Lightmap::extract(light, 3), minIntensity) / 15.0f
    };
}

inline glm::vec4 do_tint(float value) {
    return {value, value, value, 1.0f};
}

void MainBatch::cube(
        const glm::vec3 &coord,
        const glm::vec3 &size,
        const UVRegion(&texfaces)[6],
        const glm::vec4 &tint,
        bool shading
) {
    const glm::vec3 X(1.0f, 0.0f, 0.0f);
    const glm::vec3 Y(0.0f, 1.0f, 0.0f);
    const glm::vec3 Z(0.0f, 0.0f, 1.0f);

    quad(
        coord + Z * size.z * 0.5f,
        X, Y, Z, glm::vec2(size.x, size.y),
        (shading ? do_tint(0.8) * tint : tint),
        glm::vec3(1.0f), texfaces[5]
    );
    quad(
        coord - Z * size.z * 0.5f,
        -X, Y, -Z, glm::vec2(size.x, size.y),
        (shading ? do_tint(0.9f) * tint : tint),
        glm::vec3(1.0f), texfaces[4]
    );
    quad(
        coord + Y * size.y * 0.5f,
        -X, Z, Y, glm::vec2(size.x, size.z),
        (shading ? do_tint(1.0f) * tint : tint),
        glm::vec3(1.0f), texfaces[3]
    );
    quad(
        coord - Y * size.y * 0.5f,
        X, Z, -Y, glm::vec2(size.x, size.z),
        (shading ? do_tint(0.7f) * tint : tint),
        glm::vec3(1.0f), texfaces[2]
    );
    quad(
        coord + X * size.x * 0.5f,
        -Z, Y, X, glm::vec2(size.z, size.y),
        (shading ? do_tint(0.8f) * tint : tint),
        glm::vec3(1.0f), texfaces[1]
    );
    quad(
        coord - X * size.x * 0.5f,
        Z, Y, -X, glm::vec2(size.z, size.y),
        (shading ? do_tint(0.9f) * tint : tint),
        glm::vec3(1.0f), texfaces[1]
    );
}
