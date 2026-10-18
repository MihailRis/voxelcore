#pragma once

#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <unordered_map>

#include "assets/assets_util.hpp"
#include "graphics/core/MeshData.hpp"

template<typename VertexStructure>
class Mesh;
template<typename InstanceStructure>
class InstanceBuffer;
class Texture;
class Chunks;
class Assets;
struct EngineSettings;

namespace model {
    struct Mesh;
    struct Model;
}

using TextureNamesMap = std::unordered_map<std::string, std::string>;

class ModelBatch {
public:
    ModelBatch(
        size_t capacity,
        const Assets& assets,
        const Chunks& chunks,
        const EngineSettings& settings
    );
    ~ModelBatch();

    void draw(
        const glm::mat4& matrix,
        const glm::vec4& tint,
        const glm::vec3& lightSampleOffset,
        const std::shared_ptr<model::Model>& model,
        const TextureNamesMap* varTextures,
        bool translucent = false
    );

    void render();

    void renderOpaque();

    void renderTranslucent(const glm::vec3& cameraPosition);

    void setLightsOffset(const glm::vec3& offset);
private:
    const Assets& assets;
    const Chunks& chunks;

    const EngineSettings& settings;
    glm::vec3 lightsOffset {};

    struct Vertex {
        glm::vec3 coord;
        glm::vec2 uv;
        glm::vec3 normal;

        static constexpr VertexAttribute ATTRIBUTES[] = {
            {VertexAttribute::Type::FLOAT, false, 3},
            {VertexAttribute::Type::FLOAT, false, 2},
            {VertexAttribute::Type::FLOAT, false, 3},
            {{}, 0}};
    };

    struct Instance {
        glm::mat4 matrix;
        glm::vec4 tint;
        glm::vec4 light;
        glm::vec4 uvRegion;
        glm::vec4 params;

        static constexpr VertexAttribute ATTRIBUTES[] = {
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {VertexAttribute::Type::FLOAT, false, 4},
            {{}, 0}};
    };

    struct GpuMesh {
        std::unique_ptr<Mesh<Vertex>> mesh;
        size_t vertexCount = 0;
    };

    struct GpuModel {
        std::weak_ptr<model::Model> model;
        std::vector<GpuMesh> meshes;
        uint64_t lastUsed = 0;
    };

    struct DrawEntry {
        const Texture* texture;
        const Mesh<Vertex>* mesh;
        glm::vec3 lightSamplePos;
        bool shading;
        Instance instance;
    };

    std::unordered_map<const model::Model*, GpuModel> models;
    std::unordered_map<const std::string*, util::TextureRegion> textures;
    std::unordered_map<std::string, util::TextureRegion> texturesByName;
    std::vector<DrawEntry> entries;
    std::vector<DrawEntry> translucentEntries;
    std::vector<size_t> order;
    std::vector<float> distances;
    std::vector<Instance> instances;

    std::unique_ptr<InstanceBuffer<Instance>> instanceBuffer;
    std::unique_ptr<Texture> blank;
    uint64_t frame = 0;

    GpuModel& upload(const std::shared_ptr<model::Model>& model);
    void collectUnused();
    void resetOrder(size_t count);
    void render(std::vector<DrawEntry>& queue);
    void finishFrame();

    const util::TextureRegion& getTexture(
        const std::string& name, const TextureNamesMap* varTextures
    );
};
