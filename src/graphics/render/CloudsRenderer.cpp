#define GLM_ENABLE_EXPERIMENTAL
#include "CloudsRenderer.hpp"

#include "assets/Assets.hpp"
#include "coders/imageio.hpp"
#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "io/io.hpp"
#include "lighting/Lightmap.hpp"
#include "maths/FastNoiseLite.h"
#include "maths/FrustumCulling.hpp"
#include "maths/voxmaths.hpp"
#include "window/Camera.hpp"
#include "world/Weather.hpp"

#include <glm/ext.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>

static debug::Logger logger("clouds-render");

static inline constexpr int MAP_SIZE = 512;
static inline constexpr float CLOUD_VOXEL_SCALE = 8.0f;
static inline constexpr float CLOUDS_SPEED = 4.0f;

class CloudsMap {
public:
    glm::ivec3 size;
    const bool* voxels;

    CloudsMap() = default;

    CloudsMap(const glm::ivec3& size, const bool* voxels)
        : size(size), voxels(voxels) {
    }

    bool isOpen(int x, int y, int z) const {
        if (x >= 0 && x < size.x && y >= 0 && y < size.y && z >= 0 && z < size.z) {
            return !voxels[vox_index(x, y, z, size.x, size.z)];
        }
        return true;
    }
};


class VolumeRenderer final {
public:
    VolumeRenderer(size_t capacity)
        : vertices(std::make_unique<ChunkVertex[]>(capacity)),
          indexBuffer(std::make_unique<uint32_t[]>(capacity * 6)),
          capacity(capacity) {
    }

    ~VolumeRenderer() = default;

    void build(CloudsMap volumeMap) {
        this->map = std::move(volumeMap);
        overflow = false;
        offset = 0;
        indexOffset = 0;
        indexCount = 0;

        int end = map.size.x * map.size.y * map.size.z;
        for (int i = 0; i < end; i++) {
            int x = i % map.size.x;
            int y = i / (map.size.z * map.size.x);
            int z = (i / map.size.x) % map.size.z;
            const auto& vox = map.voxels[i];
            if (!vox) {
                continue;
            }
            cube({x, y, z});
            if (overflow) {
                return;
            }
        }
    }

    MeshData<ChunkVertex> createMesh() const {
        return MeshData(
            util::Buffer(vertices.get(), offset),
            std::vector<util::Buffer<uint32_t>> {
                util::Buffer(indexBuffer.get(), indexCount),
            },
            util::Buffer(
                ChunkVertex::ATTRIBUTES,
                sizeof(ChunkVertex::ATTRIBUTES) / sizeof(VertexAttribute)
            )
        );
    }

private:
    std::unique_ptr<ChunkVertex[]> vertices;
    std::unique_ptr<uint32_t[]> indexBuffer;
    size_t capacity;
    uint32_t indexOffset = 0;
    uint32_t indexCount = 0;
    uint32_t offset = 0;
    bool overflow = false;
    CloudsMap map {};

    void vertex(
        const glm::vec3& coord,
        const glm::vec3& normal
    ) {
        auto& vert = vertices[offset++];
        vert.position = coord;
        vert.uv = {};
        vert.normal = {
            static_cast<uint8_t>(normal.r * 127 + 128),
            static_cast<uint8_t>(normal.g * 127 + 128),
            static_cast<uint8_t>(normal.b * 127 + 128),
            255
        };
        vert.color = {
            0, 0, 0, static_cast<uint8_t>((coord.y / 8.0f * 0.25f + 0.75f) * 255)
        };
    }

    void face(
        const glm::vec3& coord,
        const glm::vec3& X,
        const glm::vec3& Y,
        const glm::vec3& Z
    ) {
        if (!isOpen(coord + Z)) {
            return;
        }
        if (offset + 4 >= capacity) {
            overflow = true;
            return;
        }

        float s = 0.5f;
        vertex(coord + (-X - Y + Z) * s, Z);
        vertex(coord + ( X - Y + Z) * s, Z);
        vertex(coord + ( X + Y + Z) * s, Z);
        vertex(coord + (-X + Y + Z) * s, Z);

        const uint32_t indices[] {0, 1, 2, 0, 2, 3};
        for (size_t i = 0; i < 6; i++) {
            indexBuffer[indexCount++] = indexOffset + indices[i];
        }
        indexOffset += 4;
    }

    bool isOpen(const glm::ivec3& pos) const {
        return pos.y < 0 || pos.y >= map.size.y ||
               map.isOpen(pos.x, pos.y, pos.z);
    }

    void cube(const glm::ivec3& coord) {
        const glm::ivec3 X(1, 0, 0);
        const glm::ivec3 Y(0, 1, 0);
        const glm::ivec3 Z(0, 0, 1);

        face(coord, X, Y, Z);
        face(coord, -X, Y, -Z);
        face(coord, X, -Z, Y);
        face(coord, X, Z, -Y);
        face(coord, -Z, Y, X);
        face(coord, Z, Y, -X);
    }
};

static void generate_heightmap(
    float* heightmap, fnl_state& state, int w, int dd, int layerid
) {
    float pi2 = glm::two_pi<float>();

    for (int lz = 0; lz < dd; lz++) {
        for (int lx = 0; lx < w; lx++) {
            float x = glm::sin(lx / static_cast<float>(w) * pi2) * w / pi2;
            float y = -glm::cos(lx / static_cast<float>(w) * pi2) * w / pi2;
            float z = lz;
            float s = 1.5f;
            auto n = fnlGetNoise3D(&state, x * s * 0.7, y * s, z * s * 0.7);
            n += fnlGetNoise3D(&state, x * s + fnlGetNoise2D(&state, x * s * 4 + 2, z * s * 4) * 2.0f, y * s, z * 3.0f) * 0.5f;
            n += fnlGetNoise3D(&state, x * s * 2, y * s * 2, z * s * 2) * 0.25f;
            n += fnlGetNoise3D(&state, x * s * 4, y * s * 4, z * s * 4) * 0.125f * 2;
            n += fnlGetNoise3D(&state, x * s * 8, y * s * 8, z * s * 8) * 0.125f * 0.5f * 2;
            n += fnlGetNoise3D(&state, x * s * 16, y * s * 16, z * s * 16) * 0.125f * 0.25f * 3;
            n = glm::max(0.0f, n);
            n += -0.1f - layerid * 0.3f;

            heightmap[lz * w + lx] = n;
        }
    }
}

static void sample_voxels(
    bool* voxels,
    const float* heightmap,
    int height,
    int segmentSize,
    int segmentX,
    int segmentZ
) {
    for (int y = 0; y < height; y++) {
        for (int z = 0; z < segmentSize; z++) {
            for (int x = 0; x < segmentSize; x++) {
                int gx = (segmentX * segmentSize) + x;
                int gz = (segmentZ * segmentSize) + z;

                float n = heightmap[gz * MAP_SIZE + gx];
                if (gz < MAP_SIZE / 2) {
                    float t = gz / static_cast<float>(MAP_SIZE / 2);
                    n = n * t +
                        heightmap[(MAP_SIZE + gz) * MAP_SIZE + gx] * (1.0f - t);
                }
                bool solid = y <= n * height && y >= (0.5f - n * 0.5f) * height;
                voxels[vox_index(x, y, z, segmentSize, segmentSize)] = solid;
            }
        }
    }
}

CloudsRenderer::CloudsRenderer() {
    VolumeRenderer volumeRenderer(1024 * 512);

    const int diameter = 4;
    const int segmentSize = MAP_SIZE / diameter;

    const int w = MAP_SIZE;
    const int h = 8;
    const int d = MAP_SIZE;
    const int dd = d * 1.5;
    auto heightmap = std::make_unique<float[]>(w * dd);

    for (int layerid = 0; layerid < 2; layerid++) {
        auto& layer = layers[layerid];
        layer.diameter = diameter;
        layer.segmentSize = segmentSize;

        fnl_state state = fnlCreateState();
        state.seed = 5265 + layerid * 3521;

        generate_heightmap(heightmap.get(), state, w, dd, layerid);

        bool voxels[segmentSize * h * segmentSize];
        for (int sz = 0; sz < diameter; sz++) {
            for (int sx = 0; sx < diameter; sx++) {
                sample_voxels(voxels, heightmap.get(), h, segmentSize, sx, sz);
                
                CloudsMap map({segmentSize, h, segmentSize}, voxels);

                volumeRenderer.build(map);
                layer.meshes.push_back(std::make_unique<Mesh<ChunkVertex>>(
                    volumeRenderer.createMesh()
                ));
            }
        }
    }
}

CloudsRenderer::~CloudsRenderer() = default;

void CloudsRenderer::draw(
    Layer& layer,
    Frustum& frustum,
    Shader& shader,
    const Camera& camera,
    float timer,
    int layerId
) {
    float scale = CLOUD_VOXEL_SCALE;
    int totalDiameter = layer.segmentSize * scale;

    int gcellX = floordiv(camera.position.x, totalDiameter);
    int gcellZ = floordiv(camera.position.z, totalDiameter);

    float speed = CLOUDS_SPEED;
    float speedX = glm::sin(layerId * 0.3f + 0.4f) * speed / (layerId + 1);
    float speedZ = -glm::cos(layerId * 0.3f + 0.4f) * speed / (layerId + 1);

    int radius = 4;

    for (int x = -radius; x <= radius; x++) {
        for (int z = -radius; z <= radius; z++) {
            int lcellX = gcellX - floordiv(glm::floor(timer * speedX), totalDiameter);
            int lcellZ = gcellZ - floordiv(glm::floor(timer * speedZ), totalDiameter);
            glm::vec3 position(
                -128 * scale + (x + gcellX) * layer.segmentSize * scale +
                    glm::mod(timer * speedX, static_cast<float>(totalDiameter)),
                250 + layerId * 200,
                -128 * scale + (z + gcellZ) * layer.segmentSize * scale +
                    glm::mod(timer * speedZ, static_cast<float>(totalDiameter))
            );
            if (glm::distance2(
                    glm::vec2(
                        position.x + totalDiameter * 0.5f,
                        position.z + totalDiameter * 0.5f
                    ),
                    glm::vec2(camera.position.x, camera.position.z)
                ) > 4e6) {
                continue;
            }
            if (!frustum.isBoxVisible(position, position + glm::vec3(layer.segmentSize * scale))) {
                continue;
            }
            auto matrix = glm::mat4(1.0f);
            matrix = glm::translate(matrix, position);
            matrix = glm::scale(matrix, glm::vec3(scale, scale, scale));
            shader.uniformMatrix("u_model", matrix);

            int lx = (x + radius + lcellX) % layer.diameter;
            int lz = (z + radius + lcellZ) % layer.diameter;
            if (lx < 0) lx += layer.diameter;
            if (lz < 0) lz += layer.diameter;

            layer.meshes[lz * layer.diameter + lx]->draw();
        }
    }
}

void CloudsRenderer::draw(
    Shader& shader,
    const Weather& weather,
    float timer,
    float fogFactor,
    const Camera& camera,
    int quality
) {
    Frustum frustum;
    frustum.update(camera.getProjView());

    shader.uniform4f("u_tint", glm::vec4(weather.cloudsTint(), 1.0f));
    shader.uniform1f("u_fogFactor", fogFactor * 0.03f);
    shader.uniform1f("u_fogCurve", 0.7f - 0.3f);

    for (int i = 0; i < std::min<int>(quality, layers.size()); i++) {
        draw(layers[i], frustum, shader, camera, timer, i);
    }
}
