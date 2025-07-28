#pragma once

#include <string>
#include <glm/vec2.hpp>

#include "util/HandlersList.hpp"

namespace dv {
    class value;
}

enum class BindType {
    BIND = 0,
    REBIND = 1
};

/// @brief Represents glfw3 keycode values.
enum class Keycode : int {
    SPACE = 32,
    APOSTROPHE = 39,
    COMMA = 44,
    MINUS = 45,
    PERIOD = 46,
    SLASH = 47,
    NUM_0 = 48,
    NUM_1 = 49,
    NUM_2 = 50,
    NUM_3 = 51,
    NUM_4 = 52,
    NUM_5 = 53,
    NUM_6 = 54,
    NUM_7 = 55,
    NUM_8 = 56,
    NUM_9 = 57,
    SEMICOLON = 59,
    EQUAL = 61,
    A = 65,
    B = 66,
    C = 67,
    D = 68,
    E = 69,
    F = 70,
    G = 71,
    H = 72,
    I = 73,
    J = 74,
    K = 75,
    L = 76,
    M = 77,
    N = 78,
    O = 79,
    P = 80,
    Q = 81,
    R = 82,
    S = 83,
    T = 84,
    U = 85,
    V = 86,
    W = 87,
    X = 88,
    Y = 89,
    Z = 90,
    LEFT_BRACKET = 91,
    BACKSLASH = 92,
    RIGHT_BRACKET = 93,
    GRAVE_ACCENT = 96,
    ESCAPE = 256,
    ENTER = 257,
    TAB = 258,
    BACKSPACE = 259,
    INSERT = 260,
    DELETE = 261,
    LEFT = 263,
    RIGHT = 262,
    DOWN = 264,
    UP = 265,
    PAGE_UP = 266,
    PAGE_DOWN = 267,
    HOME = 268,
    END = 269,
    CAPS_LOCK = 280,
    NUM_LOCK = 282,
    PRINT_SCREEN = 283,
    PAUSE = 284,
    F1 = 290,
    F2 = 291,
    F3 = 292,
    F4 = 293,
    F5 = 294,
    F6 = 295,
    F7 = 296,
    F8 = 297,
    F9 = 298,
    F10 = 299,
    F11 = 300,
    F12 = 301,
    LEFT_SHIFT = 340,
    LEFT_CONTROL = 341,
    LEFT_ALT = 342,
    LEFT_SUPER = 343,
    RIGHT_SHIFT = 344,
    RIGHT_CONTROL = 345,
    RIGHT_ALT = 346,
    RIGHT_SUPER = 347,
    MENU = 348,
    UNKNOWN = -1
};

/// @brief Represents glfw3 mouse button IDs.
/// @details There is a subset of glfw3 mouse button IDs.
enum class Mousecode : int {
    BUTTON_1 = 0,  // Left mouse button
    BUTTON_2 = 1,  // Right mouse button
    BUTTON_3 = 2,  // Middle mouse button
    BUTTON_4 = 3,  // Side mouse button 1
    BUTTON_5 = 4,  // Side mouse button 2
    BUTTON_6 = 5,  // Side mouse button 3
    BUTTON_7 = 6,  // Side mouse button 4
    BUTTON_8 = 7,  // Side mouse button 5
    UNKNOWN = -1,
};

inline Mousecode MOUSECODES_ALL[] {
    Mousecode::BUTTON_1,
    Mousecode::BUTTON_2,
    Mousecode::BUTTON_3,
    Mousecode::BUTTON_4,
    Mousecode::BUTTON_5,
    Mousecode::BUTTON_6,
    Mousecode::BUTTON_7,
    Mousecode::BUTTON_8};

namespace input_util {
    void initialize();

    Keycode keycode_from(const std::string& name);
    Mousecode mousecode_from(const std::string& name);

    /// @return Key label by keycode
    std::string to_string(Keycode code);
    /// @return Mouse button label by keycode
    std::string to_string(Mousecode code);

    /// @return Key name by keycode
    std::string get_name(Keycode code);
    /// @return Mouse button name by keycode
    std::string get_name(Mousecode code);
}

enum class InputType {
    KEYBOARD,
    MOUSE,
};

struct Binding {
    util::HandlersList<> onactived;

    InputType type;
    int code;
    bool state = false;
    bool justChanged = false;
    bool enabled = true;

    Binding() = default;
    Binding(InputType type, int code) : type(type), code(code) {
    }

    bool active() const {
        return state;
    }

    bool jactive() const {
        return state && justChanged;
    }

    void reset(InputType, int);
    void reset(Keycode);
    void reset(Mousecode);

    inline std::string text() const {
        switch (type) {
            case InputType::KEYBOARD: {
                return input_util::to_string(static_cast<Keycode>(code));
            }
            case InputType::MOUSE: {
                return input_util::to_string(static_cast<Mousecode>(code));
            }
        }
        return "<unknown input type>";
    }
};

class Bindings {
    std::unordered_map<std::string, Binding> bindings;
public:
    bool active(const std::string& name) const {
        const auto& found = bindings.find(name);
        if (found == bindings.end()) {
            return false;
        }
        return found->second.active();
    }

    bool jactive(const std::string& name) const {
        const auto& found = bindings.find(name);
        if (found == bindings.end()) {
            return false;
        }
        return found->second.jactive();
    }

    Binding* get(const std::string& name) {
        const auto found = bindings.find(name);
        if (found == bindings.end()) {
            return nullptr;
        }
        return &found->second;
    }

    const Binding* get(const std::string& name) const {
        const auto found = bindings.find(name);
        if (found == bindings.end()) {
            return nullptr;
        }
        return &found->second;
    }

    Binding& require(const std::string& name);

    const Binding& require(const std::string& name) const;

    void bind(const std::string& name, InputType type, int code) {
        bindings.try_emplace(name, Binding(type, code));
    }

    void rebind(const std::string& name, InputType type, int code) {
        require(name).reset(type, code);
    }

    auto& getAll() {
        return bindings;
    }

    void enableAll() {
        for (auto& entry : bindings) {
            entry.second.enabled = true;
        }
    }

    void read(const dv::value& map, BindType bindType);
    std::string write() const;
};

struct CursorState {
    bool locked = false;
    glm::vec2 pos {};
    glm::vec2 delta {};
};

class Input {
public:
    virtual ~Input() = default;

    virtual void pollEvents() = 0;

    virtual const char* getClipboardText() const = 0;
    virtual void setClipboardText(const char* str) = 0;

    virtual int getScroll() = 0;

    virtual bool pressed(Keycode keycode) const = 0;
    virtual bool jpressed(Keycode keycode) const = 0;

    virtual bool clicked(Mousecode mousecode) const = 0;
    virtual bool jclicked(Mousecode mousecode) const = 0;

    virtual CursorState getCursor() const = 0;

    virtual bool isCursorLocked() const = 0;
    virtual void toggleCursor() = 0;

    virtual Bindings& getBindings() = 0;

    virtual const Bindings& getBindings() const = 0;

    virtual ObserverHandler addKeyCallback(Keycode key, KeyCallback callback) = 0;

    virtual const std::vector<Keycode>& getPressedKeys() const = 0;
    virtual const std::vector<uint>& getCodepoints() const = 0;

    ObserverHandler addCallback(const std::string& name, KeyCallback callback) {
        return getBindings().require(name).onactived.add(callback);
    }
};
