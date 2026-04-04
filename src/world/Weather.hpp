#pragma once

#include <glm/glm.hpp>
#include <string>

#include "presets/WeatherPreset.hpp"

struct Weather : Serializable {
    WeatherPreset a {};
    WeatherPreset b {};
    std::string nameA;
    std::string nameB;
    float t = 1.0f;
    float speed = 0.0f;
    glm::vec3 highlight {}; 

    void update(float delta) {
        t += delta * speed;
        t = std::min(t, 1.0f);
        b.intensity = t;
        a.intensity = 1.0f - t;
    }

    void change(WeatherPreset preset, float time, std::string name="") {
        std::swap(a, b);
        std::swap(nameA, nameB);
        b = std::move(preset);
        t = 0.0f;
        speed = 1.0f / std::max(time, 1.e-5f);
        nameB = std::move(name);
        update(0.0f);
    }

    float fogOpacity() const {
        return b.fogOpacity * t + a.fogOpacity * (1.0f - t);
    }

    float fogDencity() const {
        return b.fogDencity * t + a.fogDencity * (1.0f - t);
    }

    float fogCurve() const {
        return b.fogCurve * t + a.fogCurve * (1.0f - t);
    }

    float thunderRate() const {
        return b.thunderRate * t + a.thunderRate * (1.0f - t);
    }

    float clouds() const {
        float sqrtT = glm::sqrt(t);
        return b.clouds * sqrtT + a.clouds * (1.0f - sqrtT);
    }

    glm::vec3 skyTint() const {
        return b.skyTint * t + a.skyTint * (1.0f - t);
    }

    glm::vec3 cloudsTint() const {
        return b.cloudsTint * t + a.cloudsTint * (1.0f - t);
    }

    glm::vec3 minSkyLight() const {
        return b.minSkyLight * t + a.minSkyLight * (1.0f - t);
    }

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};
