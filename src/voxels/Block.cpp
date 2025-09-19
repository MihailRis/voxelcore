#include "Block.hpp"

#include <set>
#include <utility>

#include "core_defs.hpp"
#include "data/StructLayout.hpp"
#include "presets/ParticlesPreset.hpp"
#include "util/stringutil.hpp"

dv::value BlockMaterial::toTable() const {
    return dv::object({
        {"name", name},
        {"stepsSound", stepsSound},
        {"placeSound", placeSound},
        {"breakSound", breakSound},
        {"hitSound", hitSound}
    });
}

dv::value BlockMaterial::serialize() const {
    return dv::object({
        {"name", name},
        {"steps-sound", stepsSound},
        {"place-sound", placeSound},
        {"break-sound", breakSound},
        {"hit-sound", hitSound}
    });
}

void BlockMaterial::deserialize(const dv::value& src) {
    src.at("name").get(name);
    src.at("steps-sound").get(stepsSound);
    src.at("place-sound").get(placeSound);
    src.at("break-sound").get(breakSound);
    src.at("hit-sound").get(hitSound);
}

CoordSystem::CoordSystem(glm::ivec3 axisX, glm::ivec3 axisY, glm::ivec3 axisZ)
    : axes({axisX, axisY, axisZ}) {
    fix = glm::ivec3(0);
    if (isVectorHasNegatives(axisX)) fix -= axisX;
    if (isVectorHasNegatives(axisY)) fix -= axisY;
    if (isVectorHasNegatives(axisZ)) fix -= axisZ;
}

void CoordSystem::transform(AABB& aabb) const {
    glm::vec3 X(axes[0]);
    glm::vec3 Y(axes[1]);
    glm::vec3 Z(axes[2]);
    aabb.a = X * aabb.a.x + Y * aabb.a.y + Z * aabb.a.z;
    aabb.b = X * aabb.b.x + Y * aabb.b.y + Z * aabb.b.z;
    aabb.a += fix;
    aabb.b += fix;
}

const BlockRotProfile BlockRotProfile::NONE {
    "none",
    {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // North
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // East
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // South
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // West
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // Up
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},  // Down
    },
    0
};

const BlockRotProfile BlockRotProfile::PIPE {
    "pipe",
    {
        {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}},    // North
        {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}},   // East
        {{-1, 0, 0}, {0, 0, -1}, {0, -1, 0}},  // South
        {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}},   // West
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},     // Up
        {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},   // Down
    },
    6
};

const BlockRotProfile BlockRotProfile::PANE {
    "pane",
    {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},    // North
        {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},   // East
        {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},  // South
        {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},   // West
    },
    4
};

const BlockRotProfile BlockRotProfile::STAIRS {
    "stairs",
    {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},    // North
        {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},   // East
        {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},  // South
        {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},   // West

        {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},
        {{0, 0, 1}, {0, -1, 0}, {1, 0, 0}},
        {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},
        {{0, 0, -1}, {0, -1, 0}, {-1, 0, 0}},
    },
    8
};

Block::Block(const std::string& name)
    : name(name), caption(util::id_to_caption(name)) {
    for (int i = 0; i < defaults.textureFaces.size(); i++) {
        defaults.textureFaces[i] = TEXTURE_NOTFOUND;
    }
}

Block::~Block() = default;

Block::Block(std::string name, const std::string& texture)
    : name(std::move(name)) {
    for (int i = 0; i < defaults.textureFaces.size(); i++) {
        defaults.textureFaces[i] = TEXTURE_NOTFOUND;
    }
}

void Block::cloneTo(Block& dst) {
    dst.caption = caption;
    for (int i = 0; i < 6; i++) {
        dst.defaults = defaults;
    }
    dst.defaults = defaults;
    if (variants) {
        dst.variants = std::make_unique<Variants>(*variants);
    }
    dst.material = material;
    std::copy(&emission[0], &emission[3], dst.emission);
    dst.size = size;
    dst.lightPassing = lightPassing;
    dst.skyLightPassing = skyLightPassing;
    dst.shadeless = shadeless;
    dst.ambientOcclusion = ambientOcclusion;
    dst.obstacle = obstacle;
    dst.selectable = selectable;
    dst.replaceable = replaceable;
    dst.breakable = breakable;
    dst.rotatable = rotatable;
    dst.grounded = grounded;
    dst.hidden = hidden;
    dst.hitboxes = hitboxes;
    dst.rotations = rotations;
    dst.pickingItem = pickingItem;
    dst.scriptName = scriptName;
    dst.uiLayout = uiLayout;
    dst.inventorySize = inventorySize;
    dst.tickInterval = tickInterval;
    dst.overlayTexture = overlayTexture;
    dst.translucent = translucent;
    dst.tags = tags;
    if (particles) {
        dst.particles = std::make_unique<ParticlesPreset>(*particles);
    }
}

static std::set<std::string, std::less<>> RESERVED_BLOCK_FIELDS {
};

bool Block::isReservedBlockField(std::string_view view) {
    return RESERVED_BLOCK_FIELDS.find(view) != RESERVED_BLOCK_FIELDS.end();
}
