#pragma once

#include <variant>
#include <stdint.h>
#include <glm/glm.hpp>

struct StructurePlacement {
    int structure;
    glm::ivec3 position;
    uint8_t rotation;

    StructurePlacement(int structure, glm::ivec3 position, uint8_t rotation)
        : structure(structure),
          position(std::move(position)),
          rotation(rotation) {
    }
};

struct LinePlacement {
    blockid_t block;
    glm::ivec3 a;
    glm::ivec3 b;
    int radius;

    LinePlacement(blockid_t block, glm::ivec3 a, glm::ivec3 b, int radius)
        : block(block), a(std::move(a)), b(std::move(b)), radius(radius) {
    }
};

struct BlockPlacement {
    blockid_t block;
    glm::ivec3 position;
    uint8_t rotation;
    bool mirror;

    BlockPlacement(blockid_t block, glm::ivec3 position, uint8_t rotation, bool mirror=false)
        : block(block), position(std::move(position)), rotation(rotation), mirror(mirror) {
    }
};

struct Placement {
    int priority;
    std::variant<StructurePlacement, LinePlacement, BlockPlacement> placement;

    Placement(
        int priority,
        std::variant<StructurePlacement, LinePlacement, BlockPlacement> placement
    ) : priority(priority), placement(std::move(placement)) {}
};
