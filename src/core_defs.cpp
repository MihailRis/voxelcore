#include "core_defs.hpp"

#include "items/ItemDef.hpp"
#include "content/Content.hpp"
#include "content/ContentBuilder.hpp"
#include "io/io.hpp"
#include "io/engine_paths.hpp"
#include "window/input.hpp"
#include "voxels/Block.hpp"
#include "coders/toml.hpp"

// All in-game definitions (blocks, items, etc..)
void corecontent::setup(Input& input, ContentBuilder& builder) {
    {
        Block& block = builder.blocks.create(CORE_AIR);
        block.replaceable = true;
        block.lightPassing = true;
        block.skyLightPassing = true;
        block.obstacle = false;
        block.selectable = false;
        block.defaults.drawGroup = 1;
        block.defaults.model.type = BlockModelType::NONE;
        block.pickingItem = CORE_EMPTY;
    }
    {
        ItemDef& item = builder.items.create(CORE_EMPTY);
        item.iconType = ItemIconType::NONE;
    }

    auto bindsFile = "res:bindings.toml";
    if (io::is_regular_file(bindsFile)) {
        input.getBindings().read(
            toml::parse(bindsFile, io::read_string(bindsFile)), BindType::BIND
        );
    }

    {
        Block& block = builder.blocks.create(CORE_OBSTACLE);
        for (uint i = 0; i < 6; i++) {
            block.defaults.textureFaces[i] = "obstacle";
        }
        block.hitboxes = {AABB()};
        block.breakable = false;
        ItemDef& item = builder.items.create(CORE_OBSTACLE+".item");
        item.iconType = ItemIconType::BLOCK;
        item.icon = CORE_OBSTACLE;
        item.placingBlock = CORE_OBSTACLE;
        item.caption = block.caption;
    }
    {
        Block& block = builder.blocks.create(CORE_STRUCT_AIR);
        for (uint i = 0; i < 6; i++) {
            block.defaults.textureFaces[i] = "struct_air";
        }
        block.defaults.drawGroup = -1;
        block.skyLightPassing = true;
        block.lightPassing = true;
        block.hitboxes = {AABB()};
        block.obstacle = false;
        ItemDef& item = builder.items.create(CORE_STRUCT_AIR+".item");
        item.iconType = ItemIconType::BLOCK;
        item.icon = CORE_STRUCT_AIR;
        item.placingBlock = CORE_STRUCT_AIR;
        item.caption = block.caption;
    }
}
