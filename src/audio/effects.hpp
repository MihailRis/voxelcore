#pragma once

#include <map>
#include <string>
#include <variant>
#include <optional>

namespace audio {
    struct Effect {
        struct Property {
            using Value = std::variant<float, int>;
            int id;
            Value def;
            Value min;
            Value max;
        };
        std::map<std::string, Property> properties;
    };

    class EffectControl {
    public:
        EffectControl(const Effect* effect) : effect(effect) {};
        virtual ~EffectControl() = default;

        virtual void set(const std::string& name, Effect::Property::Value value) = 0;
    protected:
        const Effect* effect = nullptr;
    };

    struct Acoustics {
        float reverbGain;
        float reverbReflectionsDelay;
        float reverbLateReflectionsDelay;
        float reverbDecayTime;
        float reverbRoomRolloff;
        float reverbAbsorption;
    };
}
