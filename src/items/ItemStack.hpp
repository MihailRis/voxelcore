#pragma once

#include "constants.hpp"
#include "typedefs.hpp"
#include "data/dv.hpp"

class ContentIndices;

class ItemStack {
    itemid_t item = ITEM_EMPTY;
    itemcount_t count = 0;
    dv::value fields = nullptr;
public:
    ItemStack() = default;

    ItemStack(itemid_t item, itemcount_t count, dv::value data=nullptr);

    void set(const ItemStack& item);
    void set(ItemStack&& item);
    void setCount(itemcount_t count);

    /// @brief Set a field in the item stack data.
    void setField(std::string_view name, dv::value value);

    /// @brief Get a field from the item stack data.
    /// @param name field name
    /// @return value pointer or nullptr if the field does not exist.
    dv::value* getField(const std::string& name) const;

    bool accepts(const ItemStack& item) const;

    /// @brief Move items from one stack to another. 
    /// If the target stack is completely filled, the source stack will be reduced.
    /// @param item source stack
    /// @param indices content indices
    void move(ItemStack& item, const ContentIndices& indices);

    void clear() {
        set(ItemStack(0, 0));
    }

    bool isEmpty() const {
        return item == ITEM_EMPTY;
    }

    itemid_t getItemId() const {
        return item;
    }

    itemcount_t getCount() const {
        return count;
    }

    const dv::value& getFields() const {
        return fields;
    }

    void setFields(dv::value&& table, bool clear) {
        if (clear) {
            fields = std::move(table);
            return;
        }
        for (auto&& [key, value] : table.asObject()) {
            fields[std::move(key)] = std::move(value);
        }
    }

    bool hasFields() const {
        return fields != nullptr;
    }
};
