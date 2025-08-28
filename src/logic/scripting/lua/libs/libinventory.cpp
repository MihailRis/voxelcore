#include "content/Content.hpp"
#include "items/Inventories.hpp"
#include "items/ItemStack.hpp"
#include "logic/BlocksController.hpp"
#include "world/Level.hpp"
#include "api_lua.hpp"

using namespace scripting;

namespace {
    void validate_itemid(itemid_t id) {
        if (id >= indices->items.count()) {
            throw std::runtime_error("invalid item id");
        }
    }

    Inventory& get_inventory(int64_t id) {
        auto inv = level->inventories->get(id);
        if (inv == nullptr) {
            throw std::runtime_error("inventory not found: " + std::to_string(id));
        }
        return *inv;
    }

    Inventory& get_inventory(int64_t id, int arg) {
        auto inv = level->inventories->get(id);
        if (inv == nullptr) {
            throw std::runtime_error(
                "inventory not found: " + std::to_string(id) + " argument " +
                std::to_string(arg)
            );
        }
        return *inv;
    }
    
    void validate_slotid(int slotid, const Inventory& inv) {
        if (static_cast<size_t>(slotid) >= inv.size()) {
            throw std::runtime_error(
                "slot index is out of range [0..inventory.size(invid)]"
            );
        }
    }

    using SlotFunc = int(lua::State*, ItemStack&);

    template <SlotFunc func>
    int wrap_slot(lua::State* L) {
        if (lua::isnoneornil(L, 1)) {
            throw std::runtime_error("inventory id is nil");
        }
        if (lua::isnoneornil(L, 2)) {
            throw std::runtime_error("slot index is nil");
        }
        auto invid = lua::tointeger(L, 1);
        auto slotid = lua::tointeger(L, 2);
        auto& inv = get_inventory(invid);
        validate_slotid(slotid, inv);
        auto& item = inv.getSlot(slotid);
        return func(L, item);
    }
}

static int l_get(lua::State* L, ItemStack& item) {
    lua::pushinteger(L, item.getItemId());
    lua::pushinteger(L, item.getCount());
    return 2;
}

static int l_set(lua::State* L, ItemStack& item) {
    auto itemid = lua::tointeger(L, 3);
    auto count = lua::tointeger(L, 4);
    auto data = lua::tovalue(L, 5);
    if (!data.isObject() && data != nullptr) {
        throw std::runtime_error("invalid data argument type (table expected)");
    }
    item.set(ItemStack(itemid, count, std::move(data)));
    return 0;
}

static int l_set_count(lua::State* L, ItemStack& item) {
    auto count = lua::tointeger(L, 3);
    if (item.getItemId() == ITEM_EMPTY) {
        return 0;
    }
    item.setCount(count);
    return 0;
}

static int l_size(lua::State* L) {
    auto invid = lua::tointeger(L, 1);
    const auto& inv = get_inventory(invid);
    return lua::pushinteger(L, inv.size());
}

static int l_add(lua::State* L) {
    auto invid = lua::tointeger(L, 1);
    auto itemid = lua::tointeger(L, 2);
    auto count = lua::tointeger(L, 3);
    auto data = lua::tovalue(L, 4);

    validate_itemid(itemid);
    if (!data.isObject() && data != nullptr) {
        throw std::runtime_error("invalid data argument type (table expected)");
    }

    auto& inv = get_inventory(invid);
    ItemStack item(itemid, count, std::move(data));
    inv.move(item, *indices);
    return lua::pushinteger(L, item.getCount());
}

static int l_get_block(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    int64_t id = blocks->createBlockInventory(x, y, z);
    return lua::pushinteger(L, id);
}

static int l_bind_block(lua::State* L) {
    auto id = lua::tointeger(L, 1);
    auto x = lua::tointeger(L, 2);
    auto y = lua::tointeger(L, 3);
    auto z = lua::tointeger(L, 4);
    blocks->bindInventory(id, x, y, z);
    return 0;
}

static int l_unbind_block(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    blocks->unbindInventory(x, y, z);
    return 0;
}

static int l_create(lua::State* L) {
    auto invsize = lua::tointeger(L, 1);
    auto inv = level->inventories->create(invsize);
    if (inv == nullptr) {
        return lua::pushinteger(L, 0);
    }
    return lua::pushinteger(L, inv->getId());
}

static int l_remove(lua::State* L) {
    auto invid = lua::tointeger(L, 1);
    level->inventories->remove(invid);
    return 0;
}

static int l_clone(lua::State* L) {
    auto id = lua::tointeger(L, 1);
    auto clone = level->inventories->clone(id);
    if (clone == nullptr) {
        return lua::pushinteger(L, 0);
    }
    return lua::pushinteger(L, clone->getId());
}

static int l_move(lua::State* L) {
    auto invAid = lua::tointeger(L, 1);
    auto slotAid = lua::tointeger(L, 2);
    auto& invA = get_inventory(invAid, 1);
    validate_slotid(slotAid, invA);

    auto invBid = lua::tointeger(L, 3);
    auto slotBid = lua::isnil(L, 4) ? -1 : lua::tointeger(L, 4);
    auto& invB = get_inventory(invBid, 3);
    auto& slot = invA.getSlot(slotAid);
    if (slotBid == -1) {
        invB.move(slot, *content->getIndices());
    } else {
        invB.move(slot, *content->getIndices(), slotBid, slotBid + 1);
    }
    return 0;
}

static int l_move_range(lua::State* L) {
    auto invAid = lua::tointeger(L, 1);
    auto slotAid = lua::tointeger(L, 2);
    auto& invA = get_inventory(invAid, 1);
    validate_slotid(slotAid, invA);

    auto invBid = lua::tointeger(L, 3);
    auto slotBegin = lua::isnoneornil(L, 4) ? -1 : lua::tointeger(L, 4);
    auto slotEnd = lua::isnoneornil(L, 5) ? -1 : lua::tointeger(L, 5) + 1;
    auto& invB = get_inventory(invBid, 3);
    auto& slot = invA.getSlot(slotAid);
    if (slotBegin == -1) {
        invB.move(slot, *content->getIndices());
    } else {
        invB.move(slot, *content->getIndices(), slotBegin, slotEnd);
    }
    return 0;
}

static int l_find_by_item(lua::State* L) {
    auto invId = lua::tointeger(L, 1); 
    auto& inv = get_inventory(invId, 1);
    integer_t blockid = lua::tointeger(L, 2);
    integer_t begin = lua::isnumber(L, 3) ? lua::tointeger(L, 3) : 0;
    integer_t end = lua::isnumber(L, 4) ? lua::tointeger(L, 4) : -1;
    integer_t minCount = lua::isnumber(L, 5) ? lua::tointeger(L, 5) : blockid != 0;
    size_t index = inv.findSlotByItem(blockid, begin, end, minCount);
    if (index == Inventory::npos) {
        return 0;
    }
    return lua::pushinteger(L, index);
}

static int l_get_data(lua::State* L, ItemStack& stack) {
    auto key = lua::require_string(L, 3);
    auto value = stack.getField(key);
    if (value == nullptr) {
        return 0;
    }
    return lua::pushvalue(L, *value);
}

static int l_get_all_data(lua::State* L, ItemStack& stack) {
    return lua::pushvalue(L, stack.getFields());
}

static int l_has_data(lua::State* L, ItemStack& stack) {
    auto key = lua::tostring(L, 3);
    if (key == nullptr) {
        return lua::pushboolean(L, stack.hasFields());
    }
    return lua::pushboolean(L, stack.getField(key) != nullptr);
}

static int l_set_data(lua::State* L, ItemStack& stack) {
    auto key = lua::require_string(L, 3);
    auto value = lua::tovalue(L, 4);
    stack.setField(key, std::move(value));
    return 0;
}

const luaL_Reg inventorylib[] = {
    {"get", lua::wrap<wrap_slot<l_get>>},
    {"set", lua::wrap<wrap_slot<l_set>>},
    {"set_count", lua::wrap<wrap_slot<l_set_count>>},
    {"size", lua::wrap<l_size>},
    {"add", lua::wrap<l_add>},
    {"move", lua::wrap<l_move>},
    {"move_range", lua::wrap<l_move_range>},
    {"find_by_item", lua::wrap<l_find_by_item>},
    {"get_block", lua::wrap<l_get_block>},
    {"bind_block", lua::wrap<l_bind_block>},
    {"unbind_block", lua::wrap<l_unbind_block>},
    {"get_data", lua::wrap<wrap_slot<l_get_data>>},
    {"set_data", lua::wrap<wrap_slot<l_set_data>>},
    {"get_all_data", lua::wrap<wrap_slot<l_get_all_data>>},
    {"has_data", lua::wrap<wrap_slot<l_has_data>>},
    {"create", lua::wrap<l_create>},
    {"remove", lua::wrap<l_remove>},
    {"clone", lua::wrap<l_clone>},
    {NULL, NULL}
};
