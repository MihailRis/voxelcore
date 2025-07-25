#include "ContentGfxCache.hpp"

#include <string>

#include "UiDocument.hpp"
#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "graphics/core/Atlas.hpp"
#include "maths/UVRegion.hpp"
#include "voxels/Block.hpp"
#include "core_defs.hpp"
#include "settings.hpp"


ContentGfxCache::ContentGfxCache(
    const Content& content,
    const Assets& assets,
    const GraphicsSettings& settings
)
    : content(content), assets(assets), settings(settings) {
    refresh();
}

static void refresh_variant(
    const Assets& assets,
    const Block& def,
    const Variant& variant,
    uint8_t variantIndex,
    std::unique_ptr<UVRegion[]>& sideregions,
    const Atlas& atlas,
    const GraphicsSettings& settings,
    std::unordered_map<blockid_t, model::Model>& models
) {
    for (uint side = 0; side < 6; side++) {
        std::string tex = variant.textureFaces[side];
        if (variant.culling == CullingMode::OPTIONAL &&
            !settings.denseRender.get() && atlas.has(tex + "_opaque")) {
            tex = tex + "_opaque";
        }
        if (atlas.has(tex)) {
            sideregions[(def.rt.id * 6 + side) * MAX_VARIANTS + variantIndex] = atlas.get(tex);
        } else if (atlas.has(TEXTURE_NOTFOUND)) {
            sideregions[(def.rt.id * 6 + side) * MAX_VARIANTS + variantIndex] = atlas.get(TEXTURE_NOTFOUND);
        }
    }
    if (variant.model.type == BlockModelType::CUSTOM) {
        auto model = assets.require<model::Model>(variant.model.name);

        for (auto& mesh : model.meshes) {
            size_t pos = mesh.texture.find(':');
            if (pos == std::string::npos) {
                continue;
            }
            if (auto region = atlas.getIf(mesh.texture.substr(pos+1))) {
                for (auto& vertex : mesh.vertices) {
                    vertex.uv = region->apply(vertex.uv);
                }
            }
        }
        models[def.rt.id] = std::move(model);
    }
}

void ContentGfxCache::refresh(const Block& def, const Atlas& atlas) {
    refresh_variant(assets, def, def.defaults, 0, sideregions, atlas, settings, models);
    if (def.variants) {
        const auto& variants = def.variants->variants;
        for (int i = 1; i < variants.size() - 1; i++) {
            refresh_variant(assets, def, variants[i], i, sideregions, atlas, settings, models);
        }
        def.variants->variants.at(0) = def.defaults;
    }
}

void ContentGfxCache::refresh() {
    auto indices = content.getIndices();
    sideregions = std::make_unique<UVRegion[]>(indices->blocks.count() * 6 * MAX_VARIANTS);
    const auto& atlas = assets.require<Atlas>("blocks");

    const auto& blocks = indices->blocks.getIterable();
    for (blockid_t i = 0; i < blocks.size(); i++) {
        refresh(*blocks[i], atlas);
    }
}

ContentGfxCache::~ContentGfxCache() = default;

const model::Model& ContentGfxCache::getModel(blockid_t id) const {
    const auto& found = models.find(id);
    if (found == models.end()) {
        throw std::runtime_error("model not found");
    }
    return found->second;
}
