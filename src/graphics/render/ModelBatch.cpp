#include "ModelBatch.hpp"

#include "assets/assets_util.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Mesh.hpp"
#include "assets/Assets.hpp"
#include "voxels/Chunks.hpp"
#include "settings.hpp"
#include "MainBatch.hpp"

#include <algorithm>

inline constexpr uint64_t COLLECT_EVERY = 600;
inline constexpr uint64_t UNUSED_FRAMES = 1200;

ModelBatch::ModelBatch(
    size_t capacity,
    const Assets& assets,
    const Chunks& chunks,
    const EngineSettings& settings
)
    : assets(assets), chunks(chunks), settings(settings) {
    entries.reserve(capacity / 36);

    const ubyte pixels[] = {255, 255, 255, 255};
    ImageData image(ImageFormat::RGBA8888, 1, 1, pixels);
    blank = Texture::from(&image);

    instanceBuffer = std::make_unique<InstanceBuffer<Instance>>();
}

ModelBatch::~ModelBatch() = default;

ModelBatch::GpuModel& ModelBatch::upload(
    const std::shared_ptr<model::Model>& model
) {
    auto& gpuModel = models[model.get()];
    bool valid = gpuModel.model.lock() == model &&
                 gpuModel.meshes.size() == model->meshes.size();
    for (size_t i = 0; valid && i < gpuModel.meshes.size(); i++) {
        valid = gpuModel.meshes[i].vertexCount ==
                model->meshes[i].vertices.size();
    }
    static_assert(sizeof(Vertex) == sizeof(model::Vertex));
    if (!valid) {
        gpuModel.meshes.clear();
        gpuModel.model = model;
        for (const auto& mesh : model->meshes) {
            gpuModel.meshes.push_back(GpuMesh {
                std::make_unique<Mesh<Vertex>>(
                    reinterpret_cast<const Vertex*>(mesh.vertices.data()),
                    mesh.vertices.size()
                ),
                mesh.vertices.size()});
        }
    }
    gpuModel.lastUsed = frame;
    return gpuModel;
}

void ModelBatch::collectUnused() {
    for (auto it = models.begin(); it != models.end();) {
        if (frame - it->second.lastUsed > UNUSED_FRAMES ||
            it->second.model.expired()) {
            it = models.erase(it);
        } else {
            ++it;
        }
    }
}

const util::TextureRegion& ModelBatch::getTexture(
    const std::string& name, const TextureNamesMap* varTextures
) {
    static const util::TextureRegion NONE {nullptr, UVRegion(0, 0, 1, 1)};
    if (varTextures && !name.empty() && name.at(0) == '$') {
        const auto& found = varTextures->find(name);
        if (found == varTextures->end()) {
            return NONE;
        }
        return getTexture(found->second, varTextures);
    }
    const auto& found = textures.find(&name);
    if (found != textures.end()) {
        return found->second;
    }
    auto byName = texturesByName.find(name);
    if (byName == texturesByName.end()) {
        byName = texturesByName.emplace(
            name, util::get_texture_region(assets, name, "blocks:notfound")
        ).first;
    }
    return textures[&name] = byName->second;
}

void ModelBatch::draw(
    const glm::mat4& matrix,
    const glm::vec4& tint,
    const glm::vec3& lightSampleOffset,
    const std::shared_ptr<model::Model>& model,
    const TextureNamesMap* varTextures
) {
    const auto& gpuModel = upload(model);
    glm::vec3 lightSamplePos =
        glm::vec3(matrix[3]) + lightsOffset + lightSampleOffset;
    for (size_t i = 0; i < model->meshes.size(); i++) {
        const auto& mesh = model->meshes[i];
        if (mesh.vertices.empty()) {
            continue;
        }
        const auto& region = getTexture(mesh.texture, varTextures);
        const auto& uv = region.region;
        entries.push_back(DrawEntry {
            region.texture ? region.texture : blank.get(),
            gpuModel.meshes[i].mesh.get(),
            lightSamplePos,
            mesh.shading,
            Instance {
                matrix,
                tint,
                glm::vec4(1, 1, 1, 0),
                glm::vec4(uv.u1, uv.v1, uv.getWidth(), uv.getHeight()),
                glm::vec4(mesh.shading ? 1.0f : 0.0f, 0, 0, 0)}});
    }
}

void ModelBatch::render() {
    frame++;
    if (entries.empty()) {
        textures.clear();
        texturesByName.clear();
        return;
    }
    bool backlight = settings.graphics.backlight.get();

    order.resize(entries.size());
    for (size_t i = 0; i < entries.size(); i++) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
        const auto& ea = entries[a];
        const auto& eb = entries[b];
        if (ea.texture != eb.texture) {
            return ea.texture < eb.texture;
        }
        return ea.mesh < eb.mesh;
    });
    instances.resize(entries.size());
    for (size_t i = 0; i < order.size(); i++) {
        auto& entry = entries[order[i]];
        if (entry.shading) {
            entry.instance.light =
                MainBatch::sampleLight(entry.lightSamplePos, chunks, backlight);
        }
        instances[i] = entry.instance;
    }

    instanceBuffer->reload(instances.data(), instances.size());

    const Texture* boundTexture = nullptr;
    size_t start = 0;
    for (size_t i = 1; i <= order.size(); i++) {
        if (i < order.size()) {
            const auto& prev = entries[order[i - 1]];
            const auto& next = entries[order[i]];
            if (prev.texture == next.texture && prev.mesh == next.mesh) {
                continue;
            }
        }
        const auto& entry = entries[order[start]];
        if (entry.texture != boundTexture) {
            boundTexture = entry.texture;
            boundTexture->bind();
        }
        entry.mesh->drawInstanced(*instanceBuffer, start, i - start);
        start = i;
    }

    entries.clear();
    textures.clear();
    texturesByName.clear();
    if (frame % COLLECT_EVERY == 0) {
        collectUnused();
    }
}

void ModelBatch::setLightsOffset(const glm::vec3& offset) {
    lightsOffset = offset;
}
