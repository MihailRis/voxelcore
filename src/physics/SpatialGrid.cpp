#include "SpatialGrid.hpp"

#include <algorithm>

inline constexpr double GRID_CELL_SIZE = 4.0;
inline constexpr double GRID_PADDING = 0.01;
inline constexpr double GRID_MAX_COORD = 1e9;
inline constexpr int GRID_MAX_CELLS = 64;

void SpatialGrid::clear() {
    cells.clear();
    oversized.clear();
}

void SpatialGrid::insert(
    uint id, const worldvec3& center, const glm::vec3& extents
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
    const worldvec3& center,
    const glm::vec3& extents,
    glm::ivec2& min,
    glm::ivec2& max
) {
    glm::dvec2 c(center.x, center.z);
    glm::dvec2 e = glm::dvec2(extents.x, extents.z) + GRID_PADDING;
    glm::dvec2 a = glm::floor((c - e) / GRID_CELL_SIZE);
    glm::dvec2 b = glm::floor((c + e) / GRID_CELL_SIZE);
    if (!(glm::abs(a.x) < GRID_MAX_COORD && glm::abs(a.y) < GRID_MAX_COORD &&
          glm::abs(b.x) < GRID_MAX_COORD && glm::abs(b.y) < GRID_MAX_COORD)) {
        return false;
    }
    if ((b.x - a.x + 1.0) * (b.y - a.y + 1.0) > GRID_MAX_CELLS) {
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
