#pragma once

#include "typedefs.hpp"

#include "maths/UVRegion.hpp"
#include "graphics/commons/Model.hpp"

#include <memory>
#include <unordered_map>

class Content;
class Assets;
class Atlas;
class Block;
struct UVRegion;
struct Variant;
struct GraphicsSettings;

class ContentGfxCache {
    static constexpr int GFXC_MAX_VARIANTS = 16;
    static constexpr int GFXC_SIDES = 6;
public:
    ContentGfxCache(
        const Content& content,
        const Assets& assets,
        const GraphicsSettings& settings
    );
    ~ContentGfxCache();

    static inline size_t getRegionIndex(
        blockid_t id, uint8_t variant, int side, bool opaque
    ) {
        return ((id * GFXC_SIDES + side) * GFXC_MAX_VARIANTS + variant) * 2 +
               opaque;
    }

    inline const UVRegion& getRegion(
        blockid_t id, uint8_t variant, int side, bool dense
    ) const {
        return sideregions[getRegionIndex(id, variant, side, !dense)];
    }

    const model::Model& getModel(
        blockid_t id, uint8_t variant, uint8_t rotation = 0
    ) const;

    void refresh(const Block& block, const Atlas& atlas);

    void refresh();

private:
    const Content& content;
    const Assets& assets;
    const GraphicsSettings& settings;

    // array of block sides uv regions (6 per block)
    std::unique_ptr<UVRegion[]> sideregions;
    std::unordered_map<uint64_t, model::Model> models;
    
    static inline uint64_t modelKey(
        blockid_t id, uint8_t variant, uint8_t rotation = 0
    ) {
        return (uint64_t(id) << 16) | (uint64_t(rotation & 0xFF) << 8) |
               uint64_t(variant & 0xFF);
    }

    void refreshVariant(
        const Block& def,
        const Variant& variant,
        uint8_t variantIndex,
        const Atlas& atlas
    );
};
