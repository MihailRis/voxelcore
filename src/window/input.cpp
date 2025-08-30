#include "input.hpp"

#include "debug/Logger.hpp"
#include "util/stringutil.hpp"
#include "data/dv.hpp"

#include <GLFW/glfw3.h>

#include <unordered_map>

#ifdef _WIN32
#include <windows.h>
#endif  // _WIN32

static debug::Logger logger("input");

static std::unordered_map<std::string, int> keycodes {
    {"enter", GLFW_KEY_ENTER},
    {"space", GLFW_KEY_SPACE},
    {"backspace", GLFW_KEY_BACKSPACE},
    {"caps-lock", GLFW_KEY_CAPS_LOCK},
    {"escape", GLFW_KEY_ESCAPE},
    {"delete", GLFW_KEY_DELETE},
    {"home", GLFW_KEY_HOME},
    {"end", GLFW_KEY_END},
    {"tab", GLFW_KEY_TAB},
    {"insert", GLFW_KEY_INSERT},
    {"page-down", GLFW_KEY_PAGE_DOWN},
    {"page-up", GLFW_KEY_PAGE_UP},
    {"left-shift", GLFW_KEY_LEFT_SHIFT},
    {"right-shift", GLFW_KEY_RIGHT_SHIFT},
    {"left-ctrl", GLFW_KEY_LEFT_CONTROL},
    {"right-ctrl", GLFW_KEY_RIGHT_CONTROL},
    {"left-alt", GLFW_KEY_LEFT_ALT},
    {"right-alt", GLFW_KEY_RIGHT_ALT},
    {"left-super", GLFW_KEY_LEFT_SUPER},
    {"right-super", GLFW_KEY_RIGHT_SUPER},
    {"grave-accent", GLFW_KEY_GRAVE_ACCENT},
    {"left", GLFW_KEY_LEFT},
    {"right", GLFW_KEY_RIGHT},
    {"down", GLFW_KEY_DOWN},
    {"up", GLFW_KEY_UP},
};

static std::unordered_map<std::string, int> mousecodes {
    {"left", GLFW_MOUSE_BUTTON_1},
    {"right", GLFW_MOUSE_BUTTON_2},
    {"middle", GLFW_MOUSE_BUTTON_3},
    {"side1", GLFW_MOUSE_BUTTON_4},
    {"side2", GLFW_MOUSE_BUTTON_5},
    {"side3", GLFW_MOUSE_BUTTON_6},
    {"side4", GLFW_MOUSE_BUTTON_7},
    {"side5", GLFW_MOUSE_BUTTON_8},
};

static std::unordered_map<int, std::string> keynames {};
static std::unordered_map<int, std::string> buttonsnames{};

std::string input_util::get_name(Mousecode code) {
    const auto found = buttonsnames.find(static_cast<int>(code));
    if (found == buttonsnames.end()) {
        return "unknown";
    }
    return found->second;
}

std::string input_util::get_name(Keycode code) {
    const auto found = keynames.find(static_cast<int>(code));
    if (found == keynames.end()) {
        return "unknown";
    }
    return found->second;
}

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

void input_util::initialize() {
    for (int i = 0; i <= 9; i++) {
        keycodes[std::to_string(i)] = GLFW_KEY_0 + i;
    }
    for (int i = 0; i < 25; i++) {
        keycodes["f" + std::to_string(i + 1)] = GLFW_KEY_F1 + i;
    }
    for (char i = 'a'; i <= 'z'; i++) {
        keycodes[std::string({i})] = GLFW_KEY_A - 'a' + i;
    }
    for (const auto& entry : keycodes) {
        keynames[entry.second] = entry.first;
    }
    for (const auto& entry : mousecodes) {
        buttonsnames[entry.second] = entry.first;
    }
}

Keycode input_util::keycode_from(const std::string& name) {
    const auto& found = keycodes.find(name);
    if (found == keycodes.end()) {
        return Keycode::UNKNOWN;
    }
    return static_cast<Keycode>(found->second);
}

Mousecode input_util::mousecode_from(const std::string& name) {
    const auto& found = mousecodes.find(name);
    if (found == mousecodes.end()) {
        return Mousecode::UNKNOWN;
    }
    return static_cast<Mousecode>(found->second);
}

std::string input_util::to_string(Keycode code) {
    int icode_repr = static_cast<int>(code);
    const char* name =
        glfwGetKeyName(icode_repr, glfwGetKeyScancode(icode_repr));
    if (name == nullptr) {
        switch (icode_repr) {
            case GLFW_KEY_TAB:
                return "Tab";
            case GLFW_KEY_LEFT_CONTROL:
                return "Left Ctrl";
            case GLFW_KEY_RIGHT_CONTROL:
                return "Right Ctrl";
            case GLFW_KEY_LEFT_ALT:
                return "Left Alt";
            case GLFW_KEY_RIGHT_ALT:
                return "Right Alt";
            case GLFW_KEY_LEFT_SHIFT:
                return "Left Shift";
            case GLFW_KEY_RIGHT_SHIFT:
                return "Right Shift";
            case GLFW_KEY_CAPS_LOCK:
                return "Caps-Lock";
            case GLFW_KEY_SPACE:
                return "Space";
            case GLFW_KEY_ESCAPE:
                return "Esc";
            case GLFW_KEY_ENTER:
                return "Enter";
            case GLFW_KEY_UP:
                return "Up";
            case GLFW_KEY_DOWN:
                return "Down";
            case GLFW_KEY_LEFT:
                return "Left";
            case GLFW_KEY_RIGHT:
                return "Right";
            case GLFW_KEY_BACKSPACE:
                return "Backspace";
            case GLFW_KEY_F1:
                return "F1";
            case GLFW_KEY_F2:
                return "F2";
            case GLFW_KEY_F3:
                return "F3";
            case GLFW_KEY_F4:
                return "F4";
            case GLFW_KEY_F5:
                return "F5";
            case GLFW_KEY_F6:
                return "F6";
            case GLFW_KEY_F7:
                return "F7";
            case GLFW_KEY_F8:
                return "F8";
            case GLFW_KEY_F9:
                return "F9";
            case GLFW_KEY_F10:
                return "F10";
            case GLFW_KEY_F11:
                return "F11";
            case GLFW_KEY_F12:
                return "F12";
            case GLFW_KEY_DELETE:
                return "Delete";
            case GLFW_KEY_HOME:
                return "Home";
            case GLFW_KEY_END:
                return "End";
            case GLFW_KEY_LEFT_SUPER:
                return "Left Super";
            case GLFW_KEY_RIGHT_SUPER:
                return "Right Super";
            case GLFW_KEY_PAGE_UP:
                return "Page Up";
            case GLFW_KEY_PAGE_DOWN:
                return "Page Down";
            case GLFW_KEY_INSERT:
                return "Insert";
            case GLFW_KEY_PRINT_SCREEN:
                return "Print Screen";
            case GLFW_KEY_NUM_LOCK:
                return "Num Lock";
            case GLFW_KEY_MENU:
                return "Menu";
            case GLFW_KEY_PAUSE:
                return "Pause";
            default:
                return "Unknown";
        }
    }
    return std::string(name);
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
        case Mousecode::BUTTON_6:
        case Mousecode::BUTTON_7:
        case Mousecode::BUTTON_8:
            return "XButton " + std::to_string(static_cast<int>(code) - 
                static_cast<int>(Mousecode::BUTTON_3));
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
                logger.error()
                    << "unknown input type: " << prefix << " (binding "
                    << util::quote(key) << ")";
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

#include "coders/toml.hpp"

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
