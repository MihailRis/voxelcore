#pragma once

#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "constants.hpp"
#include "data/dv.hpp"
#include "typedefs.hpp"
#include "Content.hpp"
#include "io/io.hpp"
#include "data/StructLayout.hpp"
#include "world/files/world_regions_fwd.hpp"


enum class ContentIssueType {
    REORDER,
    MISSING,
    REGION_FORMAT_UPDATE,
    BLOCK_DATA_LAYOUTS_UPDATE,
};

struct ContentIssue {
    ContentIssueType issueType;
    union {
        ContentType contentType;
        RegionLayerIndex regionLayer;
    };
};

struct ContentEntry {
    ContentType type;
    std::string name;
};

class WorldFiles;

/// @brief Content unit lookup table
/// @tparam T index type
/// @tparam U unit class
template <typename T, class U>
class ContentUnitLUT {
    std::vector<T> indices;
    std::vector<std::string> names;
    bool missingContent = false;
    bool reorderContent = false;
    /// @brief index that will be used to mark missing unit
    T missingValue;
    ContentType type;
public:
    ContentUnitLUT(
        size_t count,
        const ContentUnitIndices<U, T>& unitIndices,
        T missingValue,
        ContentType type
    )
        : missingValue(missingValue), type(type) {
        for (size_t i = 0; i < count; i++) {
            indices.push_back(i);
        }
        for (auto unit : unitIndices.getIterable()) {
            names.push_back(unit->name); 
        }
        for (size_t i = unitIndices.count(); i < count; i++) {
            names.emplace_back("");
        }
    }
    void setup(const dv::value& list, const ContentUnitDefs<U>& defs) {
        if (list != nullptr) {
            for (size_t i = 0; i < list.size(); i++) {
                const std::string& name = list[i].asString();
                if (auto def = defs.find(name)) {
                    set(i, name, def->rt.id);
                } else {
                    set(i, name, missingValue);
                }
            }
        }
    }
    void getMissingContent(std::vector<ContentEntry>& entries) const {
        for (size_t i = 0; i < count(); i++) {
            if (indices[i] == missingValue) {
                auto& name = names[i];
                entries.push_back(ContentEntry {type, name});
            }
        }
    }
    inline const std::string& getName(T index) const {
        return names[index];
    }
    inline T getId(T index) const {
        return indices[index];
    }
    inline void set(T index, std::string name, T id) {
        indices[index] = id;
        names[index] = std::move(name);
        if (id == missingValue) {
            missingContent = true;
        } else if (index != id) {
            reorderContent = true;
        }
    }
    inline ContentType getContentType() const {
        return type;
    }
    inline size_t count() const {
        return indices.size();
    }
    inline bool hasContentReorder() const {
        return reorderContent;
    }
    inline bool hasMissingContent() const {
        return missingContent;
    }
};

/// @brief Content incapatibility report used to convert world.
/// Building with indices.json
class ContentReport {
public:
    ContentUnitLUT<blockid_t, Block> blocks;
    ContentUnitLUT<itemid_t, ItemDef> items;
    uint regionsVersion;

    std::unordered_map<std::string, data::StructLayout> blocksDataLayouts;
    std::vector<ContentIssue> issues;
    std::vector<std::string> dataLoss;

    bool dataLayoutsUpdated = false;

    ContentReport(
        const ContentIndices* indices, 
        size_t blocks, 
        size_t items,
        uint regionsVersion
    );

    static std::shared_ptr<ContentReport> create(
        const std::shared_ptr<WorldFiles>& worldFiles,
        const io::path& filename,
        const Content* content
    );

    inline const std::vector<std::string>& getDataLoss() const {
        return dataLoss;
    }
    inline bool hasUpdatedLayouts() {
        return dataLayoutsUpdated;
    }

    inline bool hasContentReorder() const {
        return blocks.hasContentReorder() || items.hasContentReorder();
    }
    inline bool hasMissingContent() const {
        return blocks.hasMissingContent() || items.hasMissingContent();
    }
    inline bool isUpgradeRequired() const {
        return regionsVersion < REGION_FORMAT_VERSION;
    }
    inline bool hasDataLoss() const {
        return !dataLoss.empty();
    }
    void buildIssues();

    const std::vector<ContentIssue>& getIssues() const;
    std::vector<ContentEntry> getMissingContent() const;
};
