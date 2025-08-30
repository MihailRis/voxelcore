#include "ModelBatch.hpp"

#include "assets/assets_util.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Texture.hpp"
#include "assets/Assets.hpp"
#include "voxels/Chunks.hpp"
#include "lighting/Lightmap.hpp"
#include "settings.hpp"
#include "MainBatch.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>

inline constexpr glm::vec3 X(1, 0, 0);
inline constexpr glm::vec3 Y(0, 1, 0);
inline constexpr glm::vec3 Z(0, 0, 1);

struct DecomposedMat4 {
    glm::vec3 scale;
    glm::mat3 rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;
};

static glm::mat4 extract_rotation(glm::mat4 matrix) {
    DecomposedMat4 decomposed = {};
    glm::quat rotation;
    glm::decompose(
        matrix, 
        decomposed.scale, 
        rotation, 
        decomposed.translation, 
        decomposed.skew, 
        decomposed.perspective
    );
    return glm::toMat3(rotation);
}

ModelBatch::ModelBatch(
    size_t capacity,
    const Assets& assets,
    const Chunks& chunks,
    const EngineSettings& settings
)
    : assets(assets),
      chunks(chunks),
      settings(settings),
      batch(std::make_unique<MainBatch>(capacity)) {}

ModelBatch::~ModelBatch() = default;

void ModelBatch::draw(
    const model::Mesh& mesh,
    const glm::mat4& matrix,
    const glm::mat3& rotation,
    glm::vec3 tint,
    const texture_names_map* varTextures,
    bool backlight
) {
    setTexture(mesh.texture, varTextures);
    size_t vcount = mesh.vertices.size();
    const auto& vertexData = mesh.vertices.data();

    glm::vec4 lights(1, 1, 1, 0);
    if (mesh.shading) {
        glm::vec3 gpos = matrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        gpos += lightsOffset;
        lights = MainBatch::sampleLight(gpos, chunks, backlight);
    }
    for (size_t i = 0; i < vcount / 3; i++) {
        batch->prepare(3);
        for (size_t j = 0; j < 3; j++) {
            const auto vert = vertexData[i * 3 + j];
            float d = 1.0f;
            auto norm = rotation * vert.normal;
            if (mesh.shading) {
                d = glm::dot(norm, SUN_VECTOR);
                d = 0.8f + d * 0.2f;
            }
            batch->vertex(
                matrix * glm::vec4(vert.coord, 1.0f),
                vert.uv,
                lights * d,
                tint,
                norm,
                mesh.shading ? 0.0f : 1.0f
            );
        }
    }
}

void ModelBatch::draw(glm::mat4 matrix,
                      glm::vec3 tint,
                      const model::Model* model,
                      const texture_names_map* varTextures) {
    for (const auto& mesh : model->meshes) {
        entries.push_back({
            matrix, extract_rotation(matrix), tint, &mesh, varTextures
        });
    }
}

void ModelBatch::render() {
    std::sort(entries.begin(), entries.end(), 
        [](const DrawEntry& a, const DrawEntry& b) {
            return a.mesh->texture < b.mesh->texture;
        }
    );
    bool backlight = settings.graphics.backlight.get();
    for (auto& entry : entries) {
        draw(
            *entry.mesh,
            entry.matrix,
            entry.rotation,
            entry.tint,
            entry.varTextures,
            backlight
        );
    }
    batch->flush();
    entries.clear();
}

void ModelBatch::setLightsOffset(const glm::vec3& offset) {
    lightsOffset = offset;
}

void ModelBatch::setTexture(const std::string& name,
                            const texture_names_map* varTextures) {
    if (varTextures && name.at(0) == '$') {
        const auto& found = varTextures->find(name);
        if (found == varTextures->end()) {
            return batch->setTexture(nullptr);
        } else {
            return setTexture(found->second, varTextures);
        }
    }
    auto region = util::get_texture_region(assets, name, "blocks:notfound");
    batch->setTexture(region.texture, region.region);
}
