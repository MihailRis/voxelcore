#pragma once

#include "assets/assets_util.hpp"
#include "voxels/Block.hpp"
#include "MainBatch.hpp"
#include "typedefs.hpp"

#include <array>
#include <string>
#include <memory>
#include <unordered_map>
#include <map>

class Assets;
class Content;
class Chunks;
class Texture;
class DrawContext;

struct BlockWrapper {
    glm::ivec3 position;
    std::array<std::string, 6> textureFaces {};
    std::array<glm::vec3, 6> tints {};
    float emission = 0.0f;

    // --- render cache ---
    util::TextureRegion texRegions[6] {};
    UVRegion uvRegions[6] {};
    BlockModelType modelType {};
    uint8_t cullingBits = 0xFF;
    uint8_t dirtySides = 0xFF;
};

class BlockWrapsRenderer {
    const Assets& assets;
    const Content& content;
    const Chunks& chunks;
    std::unique_ptr<MainBatch> batch;

    std::multimap<const Texture*, BlockWrapper*> renderOrder;
    std::unordered_map<u64id_t, std::unique_ptr<BlockWrapper>> wrappers;
    u64id_t nextWrapper = 1;

    void draw(BlockWrapper& wrapper, const Texture* texture);

    void refreshWrapper(BlockWrapper& wrapper);
    void clearOrder(const BlockWrapper* const wrapper);
public:
    BlockWrapsRenderer(
        const Assets& assets, const Content& content, const Chunks& chunks
    );
    ~BlockWrapsRenderer();

    void draw(const DrawContext& ctx);

    u64id_t add(
        const glm::ivec3& position,
        const std::string& texture,
        const glm::vec3& tint,
        float emission
    );

    BlockWrapper* get(u64id_t id) const;

    void remove(u64id_t id);
};
