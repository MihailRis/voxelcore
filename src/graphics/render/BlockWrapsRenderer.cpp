#include "BlockWrapsRenderer.hpp"

#include "assets/Assets.hpp"
#include "constants.hpp"
#include "content/Content.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/render/MainBatch.hpp"
#include "lighting/Lightmap.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunks.hpp"

#include <algorithm>

BlockWrapsRenderer::BlockWrapsRenderer(
    const Assets& assets, const Content& content, const Chunks& chunks
)
    : assets(assets),
      content(content),
      chunks(chunks),
      batch(std::make_unique<MainBatch>(1024)) {
}

BlockWrapsRenderer::~BlockWrapsRenderer() = default;

void BlockWrapsRenderer::draw(BlockWrapper& wrapper, const Texture* texture) {
    if (wrapper.cullingBits == 0x0) {
        return;
    }

    uint8_t cullingBits = wrapper.cullingBits;
    for (int i = 0; i < 6; i++) {
        if ((cullingBits & (1 << i)) == 0x0) {
            continue;
        }
        cullingBits &= ~((wrapper.texRegions[i].texture != texture) << i);
    }
    if (cullingBits == 0x0) {
        return;
    }

    const voxel* vox = chunks.get(wrapper.position);
    if (vox == nullptr || vox->id == BLOCK_AIR) {
        return;
    }
    // one frame can be invalid due to texture change but ok
    const auto& def = content.getIndices()->blocks.require(vox->id);

    if (wrapper.modelType != def.getModel(vox->state.userbits).type) {
        wrapper.dirtySides = 0xFF;
    }
    glm::vec4 light(1, 1, 1, 0);
    if (wrapper.emission < 1.0f) {
        light = Lightmap::extractNormalized(chunks.getLight(wrapper.position));
        light.r += wrapper.emission;
        light.g += wrapper.emission;
        light.b += wrapper.emission;
    }
    switch (wrapper.modelType) {
        case BlockModelType::BLOCK:
            batch->cube(
                glm::vec3(wrapper.position) + glm::vec3(0.5f),
                glm::vec3(1.01f),
                wrapper.uvRegions,
                light,
                wrapper.tints.data(),
                wrapper.emission,
                cullingBits
            );
            break;
        case BlockModelType::AABB: {
            const auto& aabb =
                (def.rotatable ? def.rt.hitboxes[vox->state.rotation]
                                : def.hitboxes).at(0);
            const auto& size = aabb.size();
            batch->cube(
                glm::vec3(wrapper.position) + aabb.center(),
                size * glm::vec3(1.01f),
                wrapper.uvRegions,
                light,
                wrapper.tints.data(),
                wrapper.emission,
                cullingBits
            );
            break;
        }
        default:
            break;
    }
}

void BlockWrapsRenderer::refreshWrapper(BlockWrapper& wrapper) {
    clearOrder(&wrapper);

    for (int i = 0; i < 6; i++) {
        if ((wrapper.cullingBits & (1 << i)) == 0) {
            continue;
        }
        auto texRegion =
            util::get_texture_region(assets, wrapper.textureFaces[i], "");
        wrapper.texRegions[i] = texRegion;
        wrapper.uvRegions[i] = texRegion.region;

        auto range = renderOrder.equal_range(texRegion.texture);
        auto it =
            std::find_if(range.first, range.second, [&wrapper](auto& elem) {
                return elem.second == &wrapper;
            });
        if (it == range.second) {
            renderOrder.emplace(texRegion.texture, &wrapper);
        }
    }
    wrapper.dirtySides = 0x0;

    const voxel* vox = chunks.get(wrapper.position);
    if (vox == nullptr || vox->id == BLOCK_AIR) {
        return;
    }
    const auto& def = content.getIndices()->blocks.require(vox->id);
    wrapper.modelType = def.getModel(vox->state.userbits).type;
    switch (wrapper.modelType) {
        case BlockModelType::AABB: {
            const auto& aabb =
                (def.rotatable ? def.rt.hitboxes[vox->state.rotation]
                                : def.hitboxes).at(0);
            const auto& size = aabb.size();
            wrapper.uvRegions[0].scale(size.z, size.y);
            wrapper.uvRegions[1].scale(size.z, size.y);
            wrapper.uvRegions[2].scale(size.x, size.z);
            wrapper.uvRegions[3].scale(size.x, size.z);
            wrapper.uvRegions[4].scale(size.x, size.y);
            wrapper.uvRegions[5].scale(size.x, size.y);
            break;
        }
        default:
            break;
    }
}

void BlockWrapsRenderer::draw(const DrawContext& pctx) {
    auto ctx = pctx.sub();

    auto& shader = assets.require<Shader>("entity");
    shader.use();
    shader.uniform1i("u_alphaClip", false);
    
    for (const auto& [_, wrapper] : wrappers) {
        if (wrapper->dirtySides) {
            refreshWrapper(*wrapper);
        }
    }
    for (const auto& [texture, wrapper] : renderOrder) {
        batch->setTexture(texture);
        draw(*wrapper, texture);
    }
    batch->flush();
}

u64id_t BlockWrapsRenderer::add(
    const glm::ivec3& position,
    const std::string& texture,
    const glm::vec3& tint,
    float emission
) {
    u64id_t id = nextWrapper++;
    wrappers[id] = std::make_unique<BlockWrapper>(BlockWrapper {
        position,
        {texture, texture, texture, texture, texture, texture},
        {tint, tint, tint, tint, tint, tint},
        emission});
    return id;
}

BlockWrapper* BlockWrapsRenderer::get(u64id_t id) const {
    const auto& found = wrappers.find(id);
    if (found == wrappers.end()) {
        return nullptr;
    }
    return found->second.get();
}

void BlockWrapsRenderer::remove(u64id_t id) {
    auto found = wrappers.find(id);
    if (found == wrappers.end()) {
        return;
    }
    clearOrder(found->second.get());
    wrappers.erase(id);
}

void BlockWrapsRenderer::clearOrder(const BlockWrapper* const wrapper) {
    auto it = renderOrder.begin();
    while (it != renderOrder.end()) {
        if (it->second == wrapper) {
            it = renderOrder.erase(it);
        } else {
            ++it;
        }
    }
}