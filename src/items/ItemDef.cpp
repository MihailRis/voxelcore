#include "ItemDef.hpp"

#include "util/stringutil.hpp"

ItemDef::ItemDef(const std::string& name) : name(name) {
    caption = util::id_to_caption(name);
    description = "";
}
void ItemDef::cloneTo(ItemDef& dst) {
    dst.caption = caption;
    dst.description = description;
    dst.stackSize = stackSize;
    dst.generated = generated;
    std::copy(&emission[0], &emission[3], dst.emission);
    dst.iconType = iconType;
    dst.icon = icon;
    dst.placingBlock = placingBlock;
    dst.scriptName = scriptName;
    dst.modelName = modelName;
    dst.uses = uses;
    dst.usesDisplay = usesDisplay;
    dst.tags = tags;
}
