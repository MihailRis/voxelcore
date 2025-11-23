#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "Content.hpp"
#include "ContentPack.hpp"
#include "items/ItemDef.hpp"
#include "objects/EntityDef.hpp"
#include "world/generator/VoxelFragment.hpp"
#include "world/generator/GeneratorDef.hpp"
#include "voxels/Block.hpp"

template <class T>
class ContentUnitBuilder {
    std::unordered_map<std::string, ContentType>& allNames;
    ContentType type;

    void checkIdentifier(const std::string& id) {
        const auto& found = allNames.find(id);
        if (found != allNames.end()) {
            throw namereuse_error(
                "name " + id + " is already used", found->second
            );
        }
    }
public:
    UptrsMap<std::string, T> defs;
    std::vector<std::string> names;

    ContentUnitBuilder(
        std::unordered_map<std::string, ContentType>& allNames, ContentType type
    )
        : allNames(allNames), type(type) {
    }

    T& create(const std::string& id, bool* created = nullptr) {
        auto found = defs.find(id);
        if (found != defs.end()) {
            if (created) *created = false;
            return *found->second;
        }
        if (created) *created = true;
        checkIdentifier(id);
        allNames[id] = type;
        names.push_back(id);
        defs[id] = std::make_unique<T>(id);
        return *defs[id];
    }
    // Only fetch existing definition, return null otherwise.
    T* get(const std::string& id) {
        auto found = defs.find(id);
        if (found != defs.end()) {
            return &*found->second;
        }
        return nullptr;
    }

    auto build() {
        return std::move(defs);
    }
};

struct TagsIndices {
    int nextIndex = 1;
    std::unordered_map<std::string, int> map;

    int add(const std::string& tag) {
        const auto& found = map.find(tag);
        if (found != map.end()) {
            return found->second;
        }
        return map[tag] = nextIndex++;
    }

    int indexOf(const std::string& tag) {
        const auto& found = map.find(tag);
        if (found == map.end()) {
            return -1;
        }
        return found->second;
    }
};

class ContentBuilder {
    UptrsMap<std::string, BlockMaterial> blockMaterials;
    UptrsMap<std::string, rigging::SkeletonConfig> skeletons;
    UptrsMap<std::string, ContentPackRuntime> packs;
    std::unordered_map<std::string, ContentType> allNames;
public:
    ContentUnitBuilder<Block> blocks {allNames, ContentType::BLOCK};
    ContentUnitBuilder<ItemDef> items {allNames, ContentType::ITEM};
    ContentUnitBuilder<EntityDef> entities {allNames, ContentType::ENTITY};
    ContentUnitBuilder<GeneratorDef> generators {allNames, ContentType::GENERATOR};
    ResourceIndicesSet resourceIndices {};
    dv::value defaults = nullptr;
    TagsIndices tags {};

    ~ContentBuilder();

    void add(std::unique_ptr<ContentPackRuntime> pack);
    void add(std::unique_ptr<rigging::SkeletonConfig> skeleton);

    BlockMaterial& createBlockMaterial(const std::string& id);

    std::unique_ptr<Content> build();
};
