#pragma once

#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "content_fwd.hpp"
#include "data/dv.hpp"

using DrawGroups = std::set<ubyte>;
template <class K, class V>
using UptrsMap = std::unordered_map<K, std::unique_ptr<V>>;

class Block;
struct BlockMaterial;
struct ItemDef;
struct EntityDef;
struct GeneratorDef;

namespace rigging {
    class SkeletonConfig;
}

class namereuse_error : public std::runtime_error {
    ContentType type;
public:
    namereuse_error(const std::string& msg, ContentType type)
        : std::runtime_error(msg), type(type) {
    }

    inline ContentType getType() const {
        return type;
    }
};

template <class T>
class ContentUnitIndices {
    std::vector<T*> defs;
public:
    ContentUnitIndices(std::vector<T*> defs) : defs(std::move(defs)) {
    }

    inline const T* get(blockid_t id) const {
        if (id >= defs.size()) {
            return nullptr;
        }
        return defs[id];
    }

    inline const T& require(blockid_t id) const {
        return *defs.at(id);
    }

    inline size_t count() const {
        return defs.size();
    }

    inline const auto& getIterable() const {
        return defs;
    }
 
    inline const T* const* getDefs() const {
        return defs.data();
    }
};

/// @brief Runtime defs cache: indices
class ContentIndices {
public:
    ContentUnitIndices<Block> blocks;
    ContentUnitIndices<ItemDef> items;
    ContentUnitIndices<EntityDef> entities;

    ContentIndices(
        ContentUnitIndices<Block> blocks,
        ContentUnitIndices<ItemDef> items,
        ContentUnitIndices<EntityDef> entities
    );
};

template <class T>
class ContentUnitDefs {
    UptrsMap<std::string, T> defs;
public:
    ContentUnitDefs(UptrsMap<std::string, T> defs) : defs(std::move(defs)) {
    }

    const T* find(const std::string& id) const {
        const auto& found = defs.find(id);
        if (found == defs.end()) {
            return nullptr;
        }
        return found->second.get();
    }
    const T& require(const std::string& id) const {
        const auto& found = defs.find(id);
        if (found == defs.end()) {
            throw std::runtime_error("missing content unit " + id);
        }
        return *found->second;
    }

    T& require(const std::string& id) {
        const auto& found = defs.find(id);
        if (found == defs.end()) {
            throw std::runtime_error("missing content unit " + id);
        }
        return *found->second;
    }

    const auto& getDefs() const {
        return defs;
    }
};

class ResourceIndices {
    std::vector<std::string> names;
    std::unordered_map<std::string, size_t> indices;
    std::unique_ptr<std::vector<dv::value>> savedData;
public:
    ResourceIndices()
        : savedData(std::make_unique<std::vector<dv::value>>()) {
    }

    static constexpr size_t MISSING = SIZE_MAX;

    void add(const std::string& name, dv::value map) {
        indices[name] = names.size();
        names.push_back(name);
        savedData->push_back(std::move(map));
    }

    void addAlias(const std::string& name, const std::string& alias) {
        size_t index = indexOf(name);
        if (index == MISSING) {
            throw std::runtime_error(
                "resource does not exists: "+name);
        }
        indices[alias] = index;
    }

    const std::string& getName(size_t index) const {
        return names.at(index);
    }

    size_t indexOf(const std::string& name) const {
        const auto& found = indices.find(name);
        if (found != indices.end()) {
            return found->second;
        }
        return MISSING;
    }

    const dv::value& getSavedData(size_t index) const {
        return savedData->at(index);
    }

    void saveData(size_t index, dv::value map) const {
        savedData->at(index) = std::move(map);
    }

    size_t size() const {
        return names.size();
    }
};

using ResourceIndicesSet = ResourceIndices[RESOURCE_TYPES_COUNT];

/// @brief Content is a definitions repository
class Content {
    std::unique_ptr<ContentIndices> indices;
    UptrsMap<std::string, ContentPackRuntime> packs;
    UptrsMap<std::string, BlockMaterial> blockMaterials;
    UptrsMap<std::string, rigging::SkeletonConfig> skeletons;
    dv::value defaults = nullptr;
    std::unordered_map<std::string, int> tags;
public:
    ContentUnitDefs<Block> blocks;
    ContentUnitDefs<ItemDef> items;
    ContentUnitDefs<EntityDef> entities;
    ContentUnitDefs<GeneratorDef> generators;
    std::unique_ptr<DrawGroups> const drawGroups;
    ResourceIndicesSet resourceIndices {};

    Content(
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
        dv::value defaults,
        std::unordered_map<std::string, int> tags
    );
    ~Content();

    inline ContentIndices* getIndices() const {
        return indices.get();
    }

    inline const ResourceIndices& getIndices(ResourceType type) const {
        return resourceIndices[static_cast<size_t>(type)];
    }

    inline const dv::value& getDefaults() const {
        return defaults;
    }

    int getTagIndex(const std::string& tag) const {
        const auto& found = tags.find(tag);
        if (found == tags.end()) {
            return -1;
        }
        return found->second;
    }

    const rigging::SkeletonConfig* getSkeleton(const std::string& id) const;
    const rigging::SkeletonConfig& requireSkeleton(const std::string& id) const;
    const BlockMaterial* findBlockMaterial(const std::string& id) const;
    const ContentPackRuntime* getPackRuntime(const std::string& id) const;
    ContentPackRuntime* getPackRuntime(const std::string& id);

    const UptrsMap<std::string, BlockMaterial>& getBlockMaterials() const;
    const UptrsMap<std::string, ContentPackRuntime>& getPacks() const;
    const UptrsMap<std::string, rigging::SkeletonConfig>& getSkeletons() const;
};
