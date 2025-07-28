#include "PrecipitationRenderer.hpp"

#include "MainBatch.hpp"
#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "lighting/Lightmap.hpp"
#include "maths/util.hpp"
#include "maths/voxmaths.hpp"
#include "util/CentredMatrix.hpp"
#include "settings.hpp"
#include "presets/WeatherPreset.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "window/Camera.hpp"
#include "world/Level.hpp"

PrecipitationRenderer::PrecipitationRenderer(
    const Assets& assets,
    const Level&,
    const Chunks& chunks,
    const GraphicsSettings*
)
    : batch(std::make_unique<MainBatch>(4096)),
      chunks(chunks),
      assets(assets) {
}

PrecipitationRenderer::~PrecipitationRenderer() = default;

int PrecipitationRenderer::getHeightAt(int x, int z) {
    int y = CHUNK_H - 1;
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    auto chunk = chunks.getChunk(cx, cz);
    if (chunk == nullptr) {
        return y;
    }
    y = chunk->top;
    x -= cx * CHUNK_W;
    z -= cz * CHUNK_D;
    while (y > 0) {
        const auto& vox = chunk->voxels[vox_index(x, y, z)];
        if (vox.id == 0) {
            y--;
            continue;
        }
        break;
    }
    return y;
}

static inline glm::vec4 light_at(const Chunks& chunks, int x, int y, int z) {
    light_t lightval = chunks.getLight(x, y, z);
    return glm::vec4(
        Lightmap::extract(lightval, 0) / 15.f,
        Lightmap::extract(lightval, 1) / 15.f,
        Lightmap::extract(lightval, 2) / 15.f,
        1
    );
}

/// @brief 'Random' vertical texture coord offset to randomize rain layers
static constexpr float K = 21.41149;
/// @brief Precipitation face size
static constexpr glm::ivec2 FACE_SIZE {1, 16};

static UVRegion calc_uv(
    const glm::vec3& pos,
    const glm::vec3& right,
    float timer,
    const WeatherPreset& weather
) {
    static util::PseudoRandom random(0);

    float scale = weather.fall.scale;

    float m = glm::sign(right.x + right.z);
    int ux = pos.x;
    int uz = pos.z;
    if (glm::abs(right.x) < glm::abs(right.z)) {
        std::swap(ux, uz);
    }
    random.setSeed(uz);
    float hspeed = (random.randFloat() * 2.0f - 1.0f) * weather.fall.hspeed;
    float u1 = ux * scale + timer * hspeed * -m;
    float v1 = timer * weather.fall.vspeed + pos.y * scale + uz * K;

    return {u1, v1, u1 + m * scale, v1 + FACE_SIZE.y * scale};
}

void PrecipitationRenderer::render(
    const Camera& camera, float delta, const WeatherPreset& weather
) {
    timer += delta;

    const int radius = 6;
    const int depth = 12;

    int x = glm::floor(camera.position.x);
    int y = glm::floor(camera.position.y);
    int z = glm::floor(camera.position.z);

    util::CentredMatrix<int, (depth + 1) * 2> heights;
    heights.setCenter(x, z);
    for (int z = heights.beginY(); z < heights.endY(); z++) {
        for (int x = heights.beginX(); x < heights.endX(); x++) {
            heights.at(x, z) = getHeightAt(x, z);
        }
    }

    batch->begin();
    auto& texture = assets.require<Texture>(weather.fall.texture);
    texture.setMipMapping(false, true);
    batch->setTexture(&texture, {});

    const struct {
        glm::vec3 right;
        glm::vec3 front;
    } faces[] {
        {{-1, 0, 0}, {0, 0, 1}},
        {{1, 0, 0}, {0, 0, -1}},
        {{0, 0, -1}, {-1, 0, 0}},
        {{0, 0, 1}, {1, 0, 0}},
    };

    bool cutBack = glm::dot(camera.up, glm::vec3(0, 1, 0)) > 0.35f * camera.getFov();
    for (const auto& face : faces) {
        if (glm::dot(camera.right, face.right) < 0.0f && cutBack) {
            continue;
        }
        for (int lx = -radius; lx <= radius; lx++) {
            for (int lz = depth; lz > 0; lz--) {
                // Position calculations
                glm::vec3 pos = face.right * static_cast<float>(lx) +
                                face.front * static_cast<float>(lz);
                pos += glm::vec3(x, 0, z);
                pos.y =
                    glm::max(y - FACE_SIZE.y / 2, heights.at(pos.x, pos.z)) +
                    FACE_SIZE.y / 2 + 1;
                pos += glm::vec3(0.5f, 0.0f, 0.5f);

                // Draw
                batch->quad(
                    pos,
                    face.right,
                    {0, 1, 0},
                    glm::cross(glm::vec3(0, 1, 0), face.right),
                    FACE_SIZE,
                    light_at(chunks, pos.x, y, pos.z),
                    glm::vec3(2.0f),
                    calc_uv(pos, face.right, timer, weather)
                );
            }
        }
    }
    batch->flush();
}
