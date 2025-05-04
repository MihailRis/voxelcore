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

using texture_names_map = std::unordered_map<std::string, std::string>;

class ModelBatch {
    const Assets& assets;
    const Chunks& chunks;

    const EngineSettings& settings;
    glm::vec3 lightsOffset {};

    static inline glm::vec3 SUN_VECTOR {0.411934f, 0.863868f, -0.279161f};

    std::unique_ptr<MainBatch> batch;

    void draw(const model::Mesh& mesh, 
              const glm::mat4& matrix, 
              const glm::mat3& rotation, 
              glm::vec3 tint,
              const texture_names_map* varTextures,
              bool backlight);

    void setTexture(const std::string& name,
                    const texture_names_map* varTextures);

    struct DrawEntry {
        glm::mat4 matrix;
        glm::mat3 rotation;
        glm::vec3 tint;
        const model::Mesh* mesh;
        const texture_names_map* varTextures;
    };
    std::vector<DrawEntry> entries;
public:
    ModelBatch(
        size_t capacity,
        const Assets& assets,
        const Chunks& chunks,
        const EngineSettings& settings
    );
    ~ModelBatch();

    void draw(glm::mat4 matrix,
              glm::vec3 tint,
              const model::Model* model,
              const texture_names_map* varTextures);
    void render();

    void setLightsOffset(const glm::vec3& offset);
};
