#define VC_ENABLE_REFLECTION

#include "Rigidbody.hpp"

#include "EntityDef.hpp"
#include "Entities.hpp"
#include "Entity.hpp"
#include "data/dv_util.hpp"
#include "logic/scripting/scripting.hpp"

dv::value Rigidbody::serialize(bool saveVelocity, bool saveBodySettings) const {
    auto bodymap = dv::object();
    if (!enabled) {
        bodymap["enabled"] = false;
    }
    if (saveVelocity) {
        bodymap["vel"] = dv::to_value(hitbox.velocity);
    }
    if (saveBodySettings) {
        bodymap["damping"] = hitbox.linearDamping;
        bodymap["type"] = BodyTypeMeta.getNameString(hitbox.type);
        if (hitbox.crouching) {
            bodymap["crouch"] = hitbox.crouching;
        }
    }
    return bodymap;
}

void Rigidbody::deserialize(const dv::value& root) {
    dv::get_vec(root, "vel", hitbox.velocity);
    std::string bodyTypeName;
    root.at("type").get(bodyTypeName);
    BodyTypeMeta.getItem(bodyTypeName, hitbox.type);
    root["crouch"].asBoolean(hitbox.crouching);
    root["damping"].asNumber(hitbox.linearDamping);
}

template <void (*callback)(const Entity&, size_t, entityid_t)>
static sensorcallback create_sensor_callback(Entities& entities) {
    return [&entities](auto entityid, auto index, auto otherid) {
        if (auto entity = entities.get(entityid)) {
            if (entity->isValid()) {
                callback(*entity, index, otherid);
            }
        }
    };
}

void Rigidbody::initialize(
    const EntityDef& def, entityid_t id, Entities& entities
) {
    sensors.resize(def.radialSensors.size() + def.boxSensors.size());
    for (auto& [i, box] : def.boxSensors) {
        SensorParams params {};
        params.aabb = box;
        sensors[i] = Sensor {
            true,
            SensorType::AABB,
            i,
            id,
            params,
            params,
            {},
            {},
            create_sensor_callback<scripting::on_sensor_enter>(entities),
            create_sensor_callback<scripting::on_sensor_exit>(entities)};
    }
    for (auto& [i, radius] : def.radialSensors) {
        SensorParams params {};
        params.radial = glm::vec4(radius);
        sensors[i] = Sensor {
            true,
            SensorType::RADIUS,
            i,
            id,
            params,
            params,
            {},
            {},
            create_sensor_callback<scripting::on_sensor_enter>(entities),
            create_sensor_callback<scripting::on_sensor_exit>(entities)};
    }
}
