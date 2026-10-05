#pragma once

#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <unordered_map>

template<typename VertexStructure> class Mesh;
class Texture;
class Chunks;
class Assets;
struct EngineSettings;
class MainBatch;

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
        const model::Model* model,
        const TextureNamesMap* varTextures
    );
    void render();

    void setLightsOffset(const glm::vec3& offset);
private:
    const Assets& assets;
    const Chunks& chunks;

    const EngineSettings& settings;
    glm::vec3 lightsOffset {};
    glm::vec3 localLightsOffset {};

    static inline glm::vec3 SUN_VECTOR {0.411934f, 0.863868f, -0.279161f};

    std::unique_ptr<MainBatch> batch;

    void draw(
        const model::Mesh& mesh,
        const glm::mat4& matrix,
        const glm::mat3& rotation,
        const glm::vec4& tint,
        const TextureNamesMap* varTextures,
        bool backlight
    );

    void setTexture(const std::string& name,
                    const TextureNamesMap* varTextures);

    struct DrawEntry {
        glm::mat4 matrix;
        glm::mat3 rotation;
        glm::vec4 tint;
        glm::vec3 lightSampleOffset;
        const model::Mesh* mesh;
        const TextureNamesMap* varTextures;
    };
    std::vector<DrawEntry> entries;
};
