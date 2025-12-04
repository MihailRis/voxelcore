#include "window/input.hpp"

#include "coders/toml.hpp"
#include "debug/Logger.hpp"
#include "util/stringutil.hpp"


debug::Logger logger("input");

void Binding::reset(InputType type, int code) {
    this->type = type;
    this->code = code;
}

void Binding::reset(Keycode code) {
    reset(InputType::KEYBOARD, static_cast<int>(code));
}

void Binding::reset(Mousecode code) {
    reset(InputType::MOUSE, static_cast<int>(code));
}

std::string input_util::to_string(Mousecode code) {
    switch (code) {
        case Mousecode::BUTTON_1:
            return "LMB";
        case Mousecode::BUTTON_2:
            return "RMB";
        case Mousecode::BUTTON_3:
            return "MMB";
        case Mousecode::BUTTON_4:
        case Mousecode::BUTTON_5:
            return "XButton " + std::to_string(
                                    static_cast<int>(code) -
                                    static_cast<int>(Mousecode::BUTTON_3)
                                );
        default:
            return "unknown button";
    }
}

const Binding& Bindings::require(const std::string& name) const {
    if (const auto found = get(name)) {
        return *found;
    }
    throw std::runtime_error("binding '" + name + "' does not exist");
}

Binding& Bindings::require(const std::string& name) {
    if (const auto found = get(name)) {
        return *found;
    }
    throw std::runtime_error("binding '" + name + "' does not exist");
}

void Bindings::read(const dv::value& map, BindType bindType) {
    for (auto& [sectionName, section] : map.asObject()) {
        for (auto& [name, value] : section.asObject()) {
            auto key = sectionName + "." + name;
            auto [prefix, codename] = util::split_at(value.asString(), ':');
            InputType type;
            int code;
            if (prefix == "key") {
                type = InputType::KEYBOARD;
                code = static_cast<int>(input_util::keycode_from(codename));
            } else if (prefix == "mouse") {
                type = InputType::MOUSE;
                code = static_cast<int>(input_util::mousecode_from(codename));
            } else {
                logger.error() << "unknown input type: " << prefix
                               << " (binding " << util::quote(key) << ")";
                continue;
            }
            if (bindType == BindType::BIND) {
                bind(key, type, code);
            } else if (bindType == BindType::REBIND) {
                rebind(key, type, code);
            }
        }
    }
}

std::string Bindings::write() const {
    auto obj = dv::object();
    for (auto& entry : bindings) {
        const auto& binding = entry.second;
        std::string value;
        switch (binding.type) {
            case InputType::KEYBOARD:
                value =
                    "key:" +
                    input_util::get_name(static_cast<Keycode>(binding.code));
                break;
            case InputType::MOUSE:
                value =
                    "mouse:" +
                    input_util::get_name(static_cast<Mousecode>(binding.code));
                break;
            default:
                throw std::runtime_error("unsupported control type");
        }
        obj[entry.first] = std::move(value);
    }
    return toml::stringify(obj);
}
