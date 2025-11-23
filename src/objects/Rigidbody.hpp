#pragma once

#include "data/dv_fwd.hpp"
#include "physics/Hitbox.hpp"

#include <vector>
#include <entt/fwd.hpp>

class Entities;
struct EntityDef;

struct Rigidbody {
    bool enabled = true;
    Hitbox hitbox;
    std::vector<Sensor> sensors;

    dv::value serialize(bool saveVelocity, bool saveBodySettings) const;
    void deserialize(const dv::value& root);

    void initialize(
        const EntityDef& def, entityid_t id, Entities& entities
    );
};
