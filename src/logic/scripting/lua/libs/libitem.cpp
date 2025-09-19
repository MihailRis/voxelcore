#include "content/Content.hpp"
#include "content/ContentLoader.hpp"
#include "content/ContentControl.hpp"
#include "items/ItemDef.hpp"
#include "api_lua.hpp"
#include "engine/Engine.hpp"

using namespace scripting;

static const ItemDef* get_item_def(lua::State* L, int idx) {
    auto indices = content->getIndices();
    auto id = lua::tointeger(L, idx);
    return indices->items.get(id);
}

static int l_name(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushstring(L, def->name);
    }
    return 0;
}

static int l_index(lua::State* L) {
    auto name = lua::require_string(L, 1);
    return lua::pushinteger(L, content->items.require(name).rt.id);
}

static int l_stack_size(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushinteger(L, def->stackSize);
    }
    return 0;
}

static int l_defs_count(lua::State* L) {
    return lua::pushinteger(L, indices->items.count());
}

static int l_get_icon(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        switch (def->iconType) {
            case ItemIconType::NONE:
                return 0;
            case ItemIconType::SPRITE:
                return lua::pushstring(L, def->icon);
            case ItemIconType::BLOCK:
                return lua::pushstring(L, "block-previews:" + def->icon);
        }
    }
    return 0;
}

static int l_caption(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushstring(L, def->caption);
    }
    return 0;
}

static int l_description(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushstring(L, def->description);
    }
    return 0;
}

static int l_placing_block(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushinteger(L, def->rt.placingBlock);
    }
    return 0;
}

static int l_model_name(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushstring(L, def->modelName);
    }
    return 0;
}

static int l_emission(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        lua::createtable(L, 4, 0);
        for (int i = 0; i < 4; ++i) {
            lua::pushinteger(L, def->emission[i]);
            lua::rawseti(L, i+1);
        }
        return 1;
    }
    return 0;
}

static int l_uses(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        return lua::pushinteger(L, def->uses);
    }
    return 0;
}

static int l_reload_script(lua::State* L) {
    auto name = lua::require_string(L, 1);
    if (content == nullptr) {
        throw std::runtime_error("content is not initialized");
    }
    auto& writeableContent = *content_control->get();
    auto& def = writeableContent.items.require(name);
    ContentLoader::reloadScript(writeableContent, def);
    return 0;
}

static int l_has_tag(lua::State* L) {
    if (auto def = get_item_def(L, 1)) {
        auto tag = lua::require_string(L, 2);
        const auto& tags = def->rt.tags;
        return lua::pushboolean(L, tags.find(content->getTagIndex(tag)) != tags.end());
    }
    return 0;
}

const luaL_Reg itemlib[] = {
    {"index", lua::wrap<l_index>},
    {"name", lua::wrap<l_name>},
    {"stack_size", lua::wrap<l_stack_size>},
    {"defs_count", lua::wrap<l_defs_count>},
    {"icon", lua::wrap<l_get_icon>},
    {"caption", lua::wrap<l_caption>},
    {"description", lua::wrap<l_description>},
    {"placing_block", lua::wrap<l_placing_block>},
    {"model_name", lua::wrap<l_model_name>},
    {"emission", lua::wrap<l_emission>},
    {"uses", lua::wrap<l_uses>},
    {"reload_script", lua::wrap<l_reload_script>},
    {"has_tag", lua::wrap<l_has_tag>},
    {NULL, NULL}
};
