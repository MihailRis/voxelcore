#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <set>

#include "data/dv.hpp"
#include "typedefs.hpp"

struct ItemFuncsSet {
    bool init : 1;
    bool on_use : 1;
    bool on_use_on_block : 1;
    bool on_block_break_by : 1;
};

enum class ItemIconType {
    NONE,    // invisible (core:empty) must not be rendered
    SPRITE,  // textured quad: icon is `atlas_name:texture_name`
    BLOCK,   // block preview: icon is string block id
};

enum class ItemUsesDisplay {
    NONE,     // uses count is not displayed
    NUMBER,   // uses count is displayed as number
    RELATION, // uses count is displayed as `remain/default` relation
    VBAR,     // uses count is displayed as vertical bar without counter
    DEFAULT = VBAR,
};

struct ItemDef {
    /// @brief Item string id (with prefix included)
    std::string const name;

    /// @brief Item name will shown in inventory
    std::string caption;

    /// @brief Item description will shown in inventory
    std::string description;

    dv::value properties = nullptr;

    /// @brief Item max stack size
    itemcount_t stackSize = 64;

    /// @brief Item is generated for other content unit (like block)
    bool generated = false;
    
    /// @brief Item light emission [r, g, b] where r,g,b in range [0..15]
    uint8_t emission[4] {0, 0, 0, 0};

    /// @brief Default item uses count
    int16_t uses = -1;

    /// @brief Item uses count display mode
    ItemUsesDisplay usesDisplay = ItemUsesDisplay::DEFAULT;

    ItemIconType iconType = ItemIconType::SPRITE;
    std::string icon = "blocks:notfound";

    std::string placingBlock = "core:air";
    std::string scriptName = name.substr(name.find(':') + 1);

    std::string modelName = name + ".model";

    std::string scriptFile;

    std::vector<std::string> tags;

    struct {
        itemid_t id;
        blockid_t placingBlock;
        ItemFuncsSet funcsset {};
        bool emissive = false;

        std::set<int> tags;
    } rt {};

    ItemDef(const std::string& name);
    ItemDef(const ItemDef&) = delete;
    void cloneTo(ItemDef& dst);
};
