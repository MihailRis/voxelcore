#pragma once

#include "typedefs.hpp"
#include "data/dv.hpp"

#include <string>

struct EntityFuncsSet {
    bool init;
    bool on_despawn;
    bool on_grounded;
    bool on_fall;
    bool on_sensor_enter;
    bool on_sensor_exit;
    bool on_save;
    bool on_aim_on;
    bool on_aim_off;
    bool on_attacked;
    bool on_used;
};

struct UserComponent {
    std::string name;
    EntityFuncsSet funcsset;
    scriptenv env;
    dv::value params;

    UserComponent(
        const std::string& name,
        EntityFuncsSet funcsset,
        scriptenv env,
        dv::value params
    )
        : name(name),
          funcsset(funcsset),
          env(std::move(env)),
          params(std::move(params)) {
    }
};

struct ScriptComponents {
    std::vector<std::unique_ptr<UserComponent>> components;

    ScriptComponents() = default;

    ScriptComponents(ScriptComponents&& other)
        : components(std::move(other.components)) {
    }
};
