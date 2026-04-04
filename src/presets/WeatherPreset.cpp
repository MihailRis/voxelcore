#include "WeatherPreset.hpp"

#include "data/dv_util.hpp"

dv::value WeatherPreset::serialize() const {
    auto root = dv::object();
    
    auto froot = dv::object();
    froot["texture"] = fall.texture;
    froot["vspeed"] = fall.vspeed;
    froot["hspeed"] = fall.hspeed;
    froot["scale"] = fall.scale;
    froot["noise"] = fall.noise;
    froot["min_opacity"] = fall.minOpacity;
    froot["max_opacity"] = fall.maxOpacity;
    froot["max_intensity"] = fall.maxIntensity;
    froot["opaque"] = fall.opaque;
    if (fall.splash) {
        froot["splash"] = fall.splash->serialize();
    }
    root["fall"] = froot;

    root["fog_opacity"] = fogOpacity;
    root["fog_dencity"] = fogDencity;
    root["fog_curve"] = fogCurve;
    root["sky_tint"] = dv::to_value(skyTint);
    root["clouds_tint"] = dv::to_value(cloudsTint);
    root["min_sky_light"] = dv::to_value(minSkyLight);
    root["clouds"] = clouds;
    root["thunder_rate"] = thunderRate;

    return root;
}

void WeatherPreset::deserialize(const dv::value& src) {
    if (src.has("fall")) {
        const auto& froot = src["fall"];
        froot.at("texture").get(fall.texture);
        froot.at("vspeed").get(fall.vspeed);
        froot.at("hspeed").get(fall.hspeed);
        froot.at("scale").get(fall.scale);
        froot.at("noise").get(fall.noise);
        froot.at("min_opacity").get(fall.minOpacity);
        froot.at("max_opacity").get(fall.maxOpacity);
        froot.at("max_intensity").get(fall.maxIntensity);
        froot.at("opaque").get(fall.opaque);
        
        if (froot.has("splash")) {
            const auto& sroot = froot["splash"];
            fall.splash = ParticlesPreset {};
            fall.splash->deserialize(sroot);
        }
    }
    src.at("fog_opacity").get(fogOpacity);
    src.at("fog_dencity").get(fogDencity);
    src.at("fog_curve").get(fogCurve);
    dv::get_vec(src, "sky_tint", skyTint);
    dv::get_vec(src, "clouds_tint", cloudsTint);
    dv::get_vec(src, "min_sky_light", minSkyLight);
    src.at("clouds").get(clouds);
    src.at("thunder_rate").get(thunderRate);
}
