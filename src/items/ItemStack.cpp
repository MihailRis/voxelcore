#include "ItemStack.hpp"

#include "content/Content.hpp"
#include "ItemDef.hpp"

ItemStack::ItemStack(itemid_t item, itemcount_t count, dv::value data)
    : item(item), count(count), fields(std::move(data)) {
}

void ItemStack::set(const ItemStack& item) {
    set(ItemStack(item));
}

void ItemStack::set(ItemStack&& item) {
    this->item = item.item;
    this->count = item.count;
    this->fields = std::move(item.fields);
    if (count == 0) {
        this->item = 0;
    }
    if (this->item == 0) {
        count = 0;
    }
}

bool ItemStack::accepts(const ItemStack& other) const {
    if (isEmpty()) {
        return true;
    }
    return item == other.getItemId() && other.fields == nullptr;
}

void ItemStack::move(ItemStack& item, const ContentIndices& indices) {
    auto& def = indices.items.require(item.getItemId());
    itemcount_t count = std::min(item.count, def.stackSize - this->count);
    if (isEmpty()) {
        set(ItemStack(item.getItemId(), count, std::move(item.fields)));
    } else {
        setCount(this->count + count);
    }
    item.setCount(item.count - count);
}

void ItemStack::setCount(itemcount_t count) {
    this->count = count;
    if (count == 0) {
        clear();
    }
}

void ItemStack::maximizeCount(const ContentIndices& indices) {
    auto& def = indices.items.require(getItemId());
    this->count = def.stackSize;
    if (count == 0) {
        clear();
    }
}

void ItemStack::setField(std::string_view name, dv::value value) {
    if (fields == nullptr) {
        if (value == nullptr) {
            return;
        }
        fields = dv::object();
    }
    if (value == nullptr) {
        fields.erase(std::string(name));
        if (fields.empty()) {
            fields = nullptr;
        }
        return;
    }
    fields[std::string(name)] = std::move(value);
}

dv::value* ItemStack::getField(const std::string& name) const {
    if (fields == nullptr) {
        return nullptr;
    }
    return fields.at(name).ptr;
}
