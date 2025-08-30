#include "ContentGfxCache.hpp"

#include <string>

#include "UiDocument.hpp"
#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "graphics/core/Atlas.hpp"
#include "maths/UVRegion.hpp"
#include "voxels/Block.hpp"
#include "debug/Logger.hpp"
#include "core_defs.hpp"
#include "settings.hpp"

static debug::Logger logger("content-gfx-cache");

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
        }
        size_t index = getRegionIndex(def.rt.id, variantIndex, side, false);
        sideregions[index] = atlas.get(tex);
        sideregions[index + 1] = atlas.get(texOpaque);
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
    refreshVariant(def, def.defaults, 0, atlas);
    if (def.variants) {
        const auto& variants = def.variants->variants;
        for (int i = 1; i < variants.size() - 1; i++) {
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

const model::Model& ContentGfxCache::getModel(blockid_t id) const {
    const auto& found = models.find(id);
    if (found == models.end()) {
        throw std::runtime_error("model not found");
    }
    return found->second;
}
