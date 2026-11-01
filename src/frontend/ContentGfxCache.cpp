#include "ContentGfxCache.hpp"

#define GLM_ENABLE_EXPERIMENTAL

#include "UiDocument.hpp"
#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "graphics/core/Atlas.hpp"
#include "maths/UVRegion.hpp"
#include "maths/util.hpp"
#include "voxels/Block.hpp"
#include "debug/Logger.hpp"
#include "core_defs.hpp"
#include "settings.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <stdexcept>

static debug::Logger logger("content-gfx-cache");

static void unrotate_triangle_uv(
    const glm::vec3 (&coords)[3], glm::vec2 (&uvs)[3], const glm::vec3& normal
) {
    glm::vec3 absN = glm::abs(normal);
    if (glm::max(absN.x, glm::max(absN.y, absN.z)) < 0.999f) {
        return;
    }
    glm::vec2 d1 = uvs[1] - uvs[0];
    glm::vec2 d2 = uvs[2] - uvs[0];

    float det = d1.x * d2.y - d2.x * d1.y;
    if (std::abs(det) < 1e-6f) {
        return;
    }
    glm::vec3 e1 = coords[1] - coords[0];
    glm::vec3 e2 = coords[2] - coords[0];
    glm::vec3 pu = (e1 * d2.y - e2 * d1.y) / det;
    glm::vec3 pv = (e2 * d1.x - e1 * d2.x) / det;
    if (glm::length2(pu) < 1e-6f || glm::length2(pv) < 1e-6f) {
        return;
    }
    pu = glm::normalize(pu);
    pv = glm::normalize(pv);

    glm::vec3 cu;
    glm::vec3 cv;
    util::uv_by_normal(normal, cu, cv);

    glm::mat2 uvTransform(
        std::round(glm::dot(pu, cu)), std::round(glm::dot(pu, cv)),
        std::round(glm::dot(pv, cu)), std::round(glm::dot(pv, cv))
    );
    if (std::abs(std::abs(glm::determinant(uvTransform)) - 1.0f) > 1e-3f) {
        return;
    }
    for (auto& uv : uvs) {
        uv = uvTransform * (uv - 0.5f) + 0.5f;
    }
}

static void unrotate_mesh_uv(model::Mesh& mesh, const CoordSystem& orient) {
    glm::vec3 X = orient.axes[0];
    glm::vec3 Y = orient.axes[1];
    glm::vec3 Z = orient.axes[2];
    for (size_t j = 0; j + 2 < mesh.vertices.size(); j += 3) {
        glm::vec3 pos[3];
        glm::vec2 uv[3];
        for (int i = 0; i < 3; i++) {
            auto origin = mesh.vertices[j + i].coord - 0.5f;
            pos[i] = origin.x * X + origin.y * Y + origin.z * Z;
            uv[i] = mesh.vertices[j + i].uv;
        }
        const auto& normal = mesh.vertices[j].normal;
        unrotate_triangle_uv(pos, uv, normal.x * X + normal.y * Y + normal.z * Z);
        for (int i = 0; i < 3; i++) {
            mesh.vertices[j + i].uv = uv[i];
        }
    }
}

ContentGfxCache::ContentGfxCache(
    const Content& content,
    const Assets& assets,
    const GraphicsSettings& settings
)
    : content(content), assets(assets), settings(settings) {
    refresh();
}

void ContentGfxCache::refreshVariant(
    const Block& def,
    const Variant& variant,
    uint8_t variantIndex,
    const Atlas& atlas
) {
    std::unordered_map<std::string, std::string> optionalTextures;
    bool denseRender = settings.denseRender.get();
    for (uint side = 0; side < 6; side++) {
        std::string tex = variant.textureFaces[side];
        std::string texOpaque = tex + "_opaque";

        if (!atlas.has(tex)) {
            tex = TEXTURE_NOTFOUND;
        }

        if (!atlas.has(texOpaque)) {
            texOpaque = tex;
        } else if (variant.culling == CullingMode::OPTIONAL && !denseRender) {
            tex = texOpaque;
            optionalTextures[variant.textureFaces[side]] = texOpaque;
        }
        size_t index = getRegionIndex(def.rt.id, variantIndex, side, false);
        sideregions[index] = atlas.get(tex);
        sideregions[index + 1] = atlas.get(texOpaque);
    }
    if (variant.model.type == BlockModelType::CUSTOM) {
        const auto& srcModel = assets.require<model::Model>(variant.model.name);

        std::vector<std::optional<UVRegion>> regions;
        regions.reserve(srcModel.meshes.size());
        for (const auto& mesh : srcModel.meshes) {
            size_t pos = mesh.texture.find(':');
            if (pos == std::string::npos) {
                regions.emplace_back(std::nullopt);
                continue;
            }
            std::string texture = mesh.texture.substr(pos+1);
            const auto& found = optionalTextures.find(texture);
            if (found != optionalTextures.end()) {
                texture = found->second;
            }
            regions.emplace_back(atlas.getIf(texture));
        }

        bool uvLock = def.uvLock && def.rotatable;
        int rotationsCount = uvLock ? def.rotations.variantsCount : 1;
        for (int rotation = 0; rotation < rotationsCount; rotation++) {
            model::Model model = srcModel;
            for (size_t i = 0; i < model.meshes.size(); i++) {
                auto& mesh = model.meshes[i];
                const auto& region = regions[i];
                if (!region) {
                    continue;
                }
                if (uvLock) {
                    unrotate_mesh_uv(mesh, def.rotations.variants[rotation]);
                }
                for (auto& vertex : mesh.vertices) {
                    vertex.uv = region->apply(vertex.uv);
                }
            }
            models[modelKey(def.rt.id, variantIndex, rotation)] = std::move(model);
        }
    }
}

void ContentGfxCache::refresh(const Block& def, const Atlas& atlas) {
    refreshVariant(def, def.defaults, 0, atlas);
    if (def.variants) {
        const auto& variants = def.variants->variants;
        for (int i = 1; i < variants.size(); i++) {
            refreshVariant(def, variants[i], i, atlas);
        }
        def.variants->variants.at(0) = def.defaults;
    }
}

void ContentGfxCache::refresh() {
    auto indices = content.getIndices();
    size_t size = indices->blocks.count() * GFXC_SIDES * GFXC_MAX_VARIANTS * 2;

    logger.info() << "uv cache size is " << (sizeof(UVRegion) * size) << " B";

    sideregions = std::make_unique<UVRegion[]>(size);
    const auto& atlas = assets.require<Atlas>("blocks");

    const auto& blocks = indices->blocks.getIterable();
    for (blockid_t i = 0; i < blocks.size(); i++) {
        refresh(*blocks[i], atlas);
    }
}

ContentGfxCache::~ContentGfxCache() = default;

const model::Model& ContentGfxCache::getModel(
    blockid_t id, uint8_t variant, uint8_t rotation
) const {
    auto found = models.find(modelKey(id, variant, rotation));
    if (found == models.end() && rotation != 0) {
        found = models.find(modelKey(id, variant, 0));
    }
    if (found == models.end()) {
        throw std::runtime_error("model not found");
    }
    return found->second;
}
