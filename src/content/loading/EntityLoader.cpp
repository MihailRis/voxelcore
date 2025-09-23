#define VC_ENABLE_REFLECTION
#include "ContentUnitLoader.hpp"

#include "../ContentBuilder.hpp"
#include "coders/json.hpp"
#include "core_defs.hpp"
#include "data/dv.hpp"
#include "debug/Logger.hpp"
#include "io/io.hpp"
#include "util/stringutil.hpp"
#include "objects/EntityDef.hpp"

static debug::Logger logger("entity-content-loader");

template<> void ContentUnitLoader<EntityDef>::loadUnit(
    EntityDef& def, const std::string& name, const io::path& file
) {
    auto root = io::read_json(file);

    if (root.has("parent")) {
        const auto& parentName = root["parent"].asString();
        auto parentDef = builder.get(parentName);
        if (parentDef == nullptr) {
            throw std::runtime_error(
                "Failed to find parent(" + parentName + ") for " + name
            );
        }
        parentDef->cloneTo(def);
    }

    if (auto found = root.at("components")) {
        for (const auto& elem : *found) {
            std::string name;
            dv::value params;
            if (elem.isObject()) {
                name = elem["name"].asString();
                if (elem.has("args")) {
                    params = elem["args"];
                }
            } else {
                name = elem.asString();
            }
            def.components.push_back(ComponentInstance {
                    std::move(name), std::move(params)});
        }
    }
    if (auto found = root.at("hitbox")) {
        const auto& arr = *found;
        def.hitbox = glm::vec3(
            arr[0].asNumber(), arr[1].asNumber(), arr[2].asNumber()
        );
    }
    if (auto found = root.at("sensors")) {
        const auto& arr = *found;
        for (size_t i = 0; i < arr.size(); i++) {
            const auto& sensorarr = arr[i];
            const auto& sensorType = sensorarr[0].asString();
            if (sensorType == "aabb") {
                def.boxSensors.emplace_back(
                    i,
                    AABB {
                        {sensorarr[1].asNumber(),
                            sensorarr[2].asNumber(),
                            sensorarr[3].asNumber()},
                        {sensorarr[4].asNumber(),
                            sensorarr[5].asNumber(),
                            sensorarr[6].asNumber()}
                    }
                );
            } else if (sensorType == "radius") {
                def.radialSensors.emplace_back(i, sensorarr[1].asNumber());
            } else {
                logger.error()
                    << name << ": sensor #" << i << " - unknown type "
                    << util::quote(sensorType);
            }
        }
    }
    root.at("save").get(def.save.enabled);
    root.at("save-skeleton-pose").get(def.save.skeleton.pose);
    root.at("save-skeleton-textures").get(def.save.skeleton.textures);
    root.at("save-body-velocity").get(def.save.body.velocity);
    root.at("save-body-settings").get(def.save.body.settings);

    std::string bodyTypeName;
    root.at("body-type").get(bodyTypeName);
    BodyTypeMeta.getItem(bodyTypeName, def.bodyType);

    root.at("skeleton-name").get(def.skeletonName);
    root.at("blocking").get(def.blocking);
}
