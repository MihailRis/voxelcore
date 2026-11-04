#pragma once

#include "typedefs.hpp"

#include <vector>
#include <cstdint>
#include <glm/glm.hpp>

class SpatialGrid {
public:
    void clear();

    void insert(uint id, const glm::vec3& center, const glm::vec3& extents);

    void build();

    template <typename Func>
    bool query(
        const glm::vec3& center, const glm::vec3& extents, const Func& func
    ) const {
        glm::ivec2 min, max;
        if (!calcRange(center, extents, min, max)) {
            return false;
        }
        for (uint id : oversized) {
            func(id);
        }
        for (int x = min.x; x <= max.x; x++) {
            for (int z = min.y; z <= max.y; z++) {
                uint64_t key = calcKey(x, z);
                for (auto it = findCell(key);
                     it != cells.end() && it->first == key;
                     ++it) {
                    func(it->second);
                }
            }
        }
        return true;
    }
private:
    using Cell = std::pair<uint64_t, uint>;

    std::vector<Cell> cells;
    std::vector<uint> oversized;

    static uint64_t calcKey(int x, int z);

    static bool calcRange(
        const glm::vec3& center,
        const glm::vec3& extents,
        glm::ivec2& min,
        glm::ivec2& max
    );

    std::vector<Cell>::const_iterator findCell(uint64_t key) const;
};
