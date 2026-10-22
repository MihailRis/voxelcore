#include "SpatialGrid.hpp"

#include <algorithm>

inline constexpr float GRID_CELL_SIZE = 4.0f;
inline constexpr float GRID_PADDING = 0.01f;
inline constexpr float GRID_MAX_COORD = 1e9f;
inline constexpr int GRID_MAX_CELLS = 64;

void SpatialGrid::clear() {
    cells.clear();
    oversized.clear();
}

void SpatialGrid::insert(
    uint id, const glm::vec3& center, const glm::vec3& extents
) {
    glm::ivec2 min, max;
    if (!calcRange(center, extents, min, max)) {
        oversized.push_back(id);
        return;
    }
    for (int x = min.x; x <= max.x; x++) {
        for (int z = min.y; z <= max.y; z++) {
            cells.emplace_back(calcKey(x, z), id);
        }
    }
}

void SpatialGrid::build() {
    std::sort(cells.begin(), cells.end());
}

uint64_t SpatialGrid::calcKey(int x, int z) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
           static_cast<uint32_t>(z);
}

bool SpatialGrid::calcRange(
    const glm::vec3& center,
    const glm::vec3& extents,
    glm::ivec2& min,
    glm::ivec2& max
) {
    glm::vec2 c(center.x, center.z);
    glm::vec2 e = glm::vec2(extents.x, extents.z) + GRID_PADDING;
    glm::vec2 a = glm::floor((c - e) / GRID_CELL_SIZE);
    glm::vec2 b = glm::floor((c + e) / GRID_CELL_SIZE);
    if (!(glm::abs(a.x) < GRID_MAX_COORD && glm::abs(a.y) < GRID_MAX_COORD &&
          glm::abs(b.x) < GRID_MAX_COORD && glm::abs(b.y) < GRID_MAX_COORD)) {
        return false;
    }
    if ((b.x - a.x + 1.0f) * (b.y - a.y + 1.0f) > GRID_MAX_CELLS) {
        return false;
    }
    min = glm::ivec2(a);
    max = glm::ivec2(b);
    return true;
}

std::vector<SpatialGrid::Cell>::const_iterator SpatialGrid::findCell(
    uint64_t key
) const {
    return std::lower_bound(
        cells.begin(),
        cells.end(),
        key,
        [](const Cell& cell, uint64_t value) { return cell.first < value; }
    );
}
