#define VC_ENABLE_REFLECTION
#include "ContentUnitLoader.hpp"
#include "ContentLoadingCommons.hpp"

#include "../ContentBuilder.hpp"
#include "coders/json.hpp"
#include "core_defs.hpp"
#include "data/dv.hpp"
#include "debug/Logger.hpp"
#include "io/io.hpp"
#include "util/stringutil.hpp"
#include "items/ItemDef.hpp"

static debug::Logger logger("item-content-loader");


template<> void ContentUnitLoader<ItemDef>::loadUnit(
    ItemDef& def, const std::string& name, const io::path& file
) {
    auto root = io::read_json(file);
    process_properties(def, name, root);
    process_tags(def, root);

    if (root.has("parent")) {
        const auto& parentName = root["parent"].asString();
        auto parentDef = builder.get(parentName);
        if (parentDef == nullptr) {
            throw std::runtime_error(
                "Failed to find parent(" + parentName + ") for " + name
            );
        }
        parentDef->cloneTo(def);
    }
    root.at("caption").get(def.caption);
    root.at("description").get(def.description);

    std::string iconTypeStr = "";
    root.at("icon-type").get(iconTypeStr);
    if (iconTypeStr == "none") {
        def.iconType = ItemIconType::NONE;
    } else if (iconTypeStr == "block") {
        def.iconType = ItemIconType::BLOCK;
    } else if (iconTypeStr == "sprite") {
        def.iconType = ItemIconType::SPRITE;
    } else if (iconTypeStr.length()) {
        logger.error() << name << ": unknown icon type - " << iconTypeStr;
    }
    root.at("icon").get(def.icon);
    root.at("placing-block").get(def.placingBlock);
    root.at("script-name").get(def.scriptName);
    root.at("model-name").get(def.modelName);
    root.at("stack-size").get(def.stackSize);
    root.at("uses").get(def.uses);

    std::string usesDisplayStr = "";
    root.at("uses-display").get(usesDisplayStr);
    if (usesDisplayStr == "none") {
        def.usesDisplay = ItemUsesDisplay::NONE;
    } else if (usesDisplayStr == "number") {
        def.usesDisplay = ItemUsesDisplay::NUMBER;
    } else if (usesDisplayStr == "relation") {
        def.usesDisplay = ItemUsesDisplay::RELATION;
    } else if (usesDisplayStr == "vbar") {
        def.usesDisplay = ItemUsesDisplay::VBAR;
    } else if (usesDisplayStr.length()) {
        logger.error() << name << ": unknown uses display mode - " << usesDisplayStr;
    }

    if (auto found = root.at("emission")) {
        const auto& emissionarr = *found;
        def.emission[0] = emissionarr[0].asNumber();
        def.emission[1] = emissionarr[1].asNumber();
        def.emission[2] = emissionarr[2].asNumber();
    }

    def.scriptFile = pack.id + ":scripts/" + def.scriptName + ".lua";
}
