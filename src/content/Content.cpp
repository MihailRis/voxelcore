#include "Content.hpp"

#include <glm/glm.hpp>
#include <memory>
#include <stdexcept>
#include <utility>

#include "items/ItemDef.hpp"
#include "logic/scripting/scripting.hpp"
#include "objects/EntityDef.hpp"
#include "objects/rigging.hpp"
#include "voxels/Block.hpp"
#include "world/generator/VoxelFragment.hpp"
#include "world/generator/GeneratorDef.hpp"
#include "ContentPack.hpp"

ContentIndices::ContentIndices(
    ContentUnitIndices<Block> blocks,
    ContentUnitIndices<ItemDef> items,
    ContentUnitIndices<EntityDef> entities
)
    : blocks(std::move(blocks)),
      items(std::move(items)),
      entities(std::move(entities)) {
}

Content::Content(
    std::unique_ptr<ContentIndices> indices,
    std::unique_ptr<DrawGroups> drawGroups,
    ContentUnitDefs<Block> blocks,
    ContentUnitDefs<ItemDef> items,
    ContentUnitDefs<EntityDef> entities,
    ContentUnitDefs<GeneratorDef> generators,
    UptrsMap<std::string, ContentPackRuntime> packs,
    UptrsMap<std::string, BlockMaterial> blockMaterials,
    UptrsMap<std::string, rigging::SkeletonConfig> skeletons,
    ResourceIndicesSet resourceIndices,
    dv::value defaults
)
    : indices(std::move(indices)),
      packs(std::move(packs)),
      blockMaterials(std::move(blockMaterials)),
      skeletons(std::move(skeletons)),
      defaults(std::move(defaults)),
      blocks(std::move(blocks)),
      items(std::move(items)),
      entities(std::move(entities)),
      generators(std::move(generators)),
      drawGroups(std::move(drawGroups)) {
    for (size_t i = 0; i < RESOURCE_TYPES_COUNT; i++) {
        this->resourceIndices[i] = std::move(resourceIndices[i]);
    }
}

Content::~Content() = default;

const rigging::SkeletonConfig* Content::getSkeleton(const std::string& id
) const {
    auto found = skeletons.find(id);
    if (found == skeletons.end()) {
        return nullptr;
    }
    return found->second.get();
}

const rigging::SkeletonConfig& Content::requireSkeleton(const std::string& id) const {
    auto skeleton = getSkeleton(id);
    if (skeleton == nullptr) {
        throw std::runtime_error("skeleton '" + id + "' not loaded");
    }
    return *skeleton;
}

const BlockMaterial* Content::findBlockMaterial(const std::string& id) const {
    auto found = blockMaterials.find(id);
    if (found == blockMaterials.end()) {
        return nullptr;
    }
    return found->second.get();
}

const ContentPackRuntime* Content::getPackRuntime(const std::string& id) const {
    auto found = packs.find(id);
    if (found == packs.end()) {
        return nullptr;
    }
    return found->second.get();
}

ContentPackRuntime* Content::getPackRuntime(const std::string& id) {
    auto found = packs.find(id);
    if (found == packs.end()) {
        return nullptr;
    }
    return found->second.get();
}

const UptrsMap<std::string, BlockMaterial>& Content::getBlockMaterials() const {
    return blockMaterials;
}

const UptrsMap<std::string, ContentPackRuntime>& Content::getPacks() const {
    return packs;
}

const UptrsMap<std::string, rigging::SkeletonConfig>& Content::getSkeletons(
) const {
    return skeletons;
}
