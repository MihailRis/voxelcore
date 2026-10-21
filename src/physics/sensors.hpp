#pragma once

#include "Hitbox.hpp"

#include <vector>

namespace physics {
    void update_sensors(Hitbox& hitbox, const std::vector<Sensor*>& sensors);
}
