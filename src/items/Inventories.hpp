#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "maths/util.hpp"
#include "Inventory.hpp"

class Level;

using inventories_map = std::unordered_map<int64_t, std::shared_ptr<Inventory>>;

/// @brief Inventories runtime storage
class Inventories {
    Level& level;
    inventories_map map;
    util::PseudoRandom random;
public:
    Inventories(Level& level);
    ~Inventories();

    /// @brief Create new inventory with new id
    std::shared_ptr<Inventory> create(size_t size);

    /// @brief Create runtime-only inventory (has negative id)
    std::shared_ptr<Inventory> createVirtual(size_t size);

    /// @brief Check and add inventory to the storage
    void store(const std::shared_ptr<Inventory>& inv);

    /// @brief Remove inventory from map
    void remove(int64_t id);

    /// @brief Get inventory by id (works with both real and virtual)
    std::shared_ptr<Inventory> get(int64_t id);

    /// @brief Create inventory non-virtual copy
    std::shared_ptr<Inventory> clone(int64_t id);

    const inventories_map& getMap() const;
};

