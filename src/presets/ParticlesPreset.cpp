#define VC_ENABLE_REFLECTION

#include "ParticlesPreset.hpp"
#include "data/dv_util.hpp"

dv::value ParticlesPreset::serialize() const {
    auto root = dv::object();
    if (frames.empty()) {
        root["texture"] = texture;
    } else {
        auto& arr = root.list("frames");
        for (const auto& frame : frames) {
            arr.add(frame);
        }
    }
    root["collision"] = collision;
    root["lighting"] = lighting;
    root["max_distance"] = maxDistance;
    root["global_up_vector"] = globalUpVector;
    root["spawn_interval"] = spawnInterval;
    root["lifetime"] = lifetime;
    root["lifetime_spread"] = lifetimeSpread;
    root["velocity"] = dv::to_value(velocity);
    root["acceleration"] = dv::to_value(acceleration);
    root["explosion"] = dv::to_value(explosion);
    root["size"] = dv::to_value(size);
    root["size_spread"] = sizeSpread;
    root["angle_spread"] = angleSpread;
    root["min_angular_vel"] = minAngularVelocity;
    root["max_angular_vel"] = maxAngularVelocity;
    root["spawn_spread"] = dv::to_value(size);
    root["spawn_shape"] = ParticleSpawnShapeMeta.getName(spawnShape);
    root["random_sub_uv"] = randomSubUV;
    return root;
}

void ParticlesPreset::deserialize(const dv::value& src) {
    src.at("texture").get(texture);
    src.at("collision").get(collision);
    src.at("lighting").get(lighting);
    src.at("global_up_vector").get(globalUpVector);
    src.at("max_distance").get(maxDistance);
    src.at("spawn_interval").get(spawnInterval);
    src.at("lifetime").get(lifetime);
    src.at("lifetime_spread").get(lifetimeSpread);
    src.at("angle_spread").get(angleSpread);
    src.at("min_angular_vel").get(minAngularVelocity);
    src.at("max_angular_vel").get(maxAngularVelocity);
    src.at("random_sub_uv").get(randomSubUV);
    if (src.has("velocity")) {
        dv::get_vec(src["velocity"], velocity);
    }
    if (src.has("acceleration")) {
        dv::get_vec(src["acceleration"], acceleration);
    }
    if (src.has("size")) {
        dv::get_vec(src["size"], size);
    }
    src.at("size_spread").get(sizeSpread);
    if (src.has("spawn_spread")) {
        dv::get_vec(src["spawn_spread"], spawnSpread);
    }
    if (src.has("explosion")) {
        dv::get_vec(src["explosion"], explosion);
    }
    if (src.has("spawn_shape")) {
        ParticleSpawnShapeMeta.getItem(src["spawn_shape"].asString(), spawnShape);
    }
    if (src.has("frames")) {
        for (const auto& frame : src["frames"]) {
            frames.push_back(frame.asString());
        }
        if (!frames.empty()) {
            texture = frames.at(0);
            randomSubUV = 1.0f;
        }
    }
}
