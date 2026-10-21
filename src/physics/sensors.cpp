#include "sensors.hpp"

void physics::update_sensors(
    Hitbox& hitbox, const std::vector<Sensor*>& sensors
) {
    auto aabb = hitbox.getAABB();

    for (size_t i = 0; i < sensors.size(); i++) {
        auto& sensor = *sensors[i];
        if (sensor.entity == hitbox.entity) {
            continue;
        }
        bool triggered = false;
        switch (sensor.type) {
            case SensorType::AABB:
                triggered = aabb.intersects(sensor.calculated.aabb);
                break;
            case SensorType::RADIUS:
                triggered = glm::distance2(
                    hitbox.position, glm::vec3(sensor.calculated.radial))
                    < sensor.calculated.radial.w;
                break;
        }
        if (!triggered) {
            continue;
        }
        if (sensor.prevEntered.find(hitbox.entity) == sensor.prevEntered.end()) {
            sensor.enterCallback(sensor.entity, sensor.index, hitbox.entity);
        }
        sensor.nextEntered.insert(hitbox.entity);
    }
}
