#include "BlockWrapsRenderer.hpp"

#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "constants.hpp"
#include "content/Content.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/render/MainBatch.hpp"
#include "objects/Player.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunks.hpp"
#include "world/Level.hpp"

BlockWrapsRenderer::BlockWrapsRenderer(
    const Assets& assets, const Level& level, const Chunks& chunks
)
    : assets(assets),
      level(level),
      chunks(chunks),
      batch(std::make_unique<MainBatch>(1024)) {
}

BlockWrapsRenderer::~BlockWrapsRenderer() = default;

void BlockWrapsRenderer::draw(const BlockWrapper& wrapper) {
    auto textureRegion = util::get_texture_region(assets, wrapper.texture, "");

    auto& shader = assets.require<Shader>("entity");
    shader.use();
    shader.uniform1i("u_alphaClip", false);

    const UVRegion& cracksRegion = textureRegion.region;
    UVRegion regions[6] {
        cracksRegion, cracksRegion, cracksRegion,
        cracksRegion, cracksRegion, cracksRegion
    };
    batch->setTexture(textureRegion.texture);

    const voxel* vox = chunks.get(wrapper.position);
    if (vox == nullptr) {
        return;
    }
    if (vox->id != BLOCK_VOID) {
        const auto& def = level.content.getIndices()->blocks.require(vox->id);
        switch (def.getModel(vox->state.userbits).type) {
            case BlockModelType::BLOCK:
                batch->cube(
                    glm::vec3(wrapper.position) + glm::vec3(0.5f),
                    glm::vec3(1.01f),
                    regions,
                    glm::vec4(1,1,1,0),
                    false
                );
                break;
            case BlockModelType::AABB: {
                const auto& aabb =
                    (def.rotatable ? def.rt.hitboxes[vox->state.rotation]
                                   : def.hitboxes)
                        .at(0);
                const auto& size = aabb.size();
                regions[0].scale(size.z, size.y);
                regions[1].scale(size.z, size.y);
                regions[2].scale(size.x, size.z);
                regions[3].scale(size.x, size.z);
                regions[4].scale(size.x, size.y);
                regions[5].scale(size.x, size.y);
                batch->cube(
                    glm::vec3(wrapper.position) + aabb.center(),
                    size * glm::vec3(1.01f),
                    regions,
                    glm::vec4(1,1,1,0),
                    false
                );
                break;
            }
            default:
                break;
        }
    }
}

void BlockWrapsRenderer::draw(const DrawContext& pctx, const Player& player) {
    auto ctx = pctx.sub();
    for (const auto& [_, wrapper] : wrappers) {
        draw(*wrapper);
    }
    batch->flush();
}

u64id_t BlockWrapsRenderer::add(
    const glm::ivec3& position, const std::string& texture
) {
    u64id_t id = nextWrapper++;
    wrappers[id] = std::make_unique<BlockWrapper>(
        BlockWrapper {position, texture}
    );
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
    wrappers.erase(id);
}
