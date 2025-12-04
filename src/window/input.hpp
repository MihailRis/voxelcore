#pragma once

#include <cstdint>
#include <glm/vec2.hpp>
#include <string>

#include "util/HandlersList.hpp"

namespace dv {
    class value;
}

enum class BindType { BIND = 0, REBIND = 1 };

/// @brief Represents sdl scancode values.
enum class Keycode : std::int32_t {
    SPACE = 44,
    APOSTROPHE = 52,
    COMMA = 54,
    MINUS = 45,
    PERIOD = 55,
    SLASH = 56,
    NUM_0 = 39,
    NUM_1 = 30,
    NUM_2,
    NUM_3,
    NUM_4,
    NUM_5,
    NUM_6,
    NUM_7,
    NUM_8,
    NUM_9,
    SEMICOLON = 51,
    EQUAL = 46,
    A = 4,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    LEFT_BRACKET = 47,
    BACKSLASH = 49,
    RIGHT_BRACKET = 48,
    GRAVE_ACCENT = 53,
    ESCAPE = 41,
    ENTER = 40,
    TAB = 43,
    BACKSPACE = 42,
    INSERT = 73,
    DELETE = 76,
    LEFT = 80,
    RIGHT = 79,
    DOWN = 81,
    UP = 82,
    PAGE_UP = 75,
    PAGE_DOWN = 78,
    HOME = 74,
    END = 77,
    CAPS_LOCK = 57,
    NUM_LOCK = 83,
    PRINT_SCREEN = 70,
    PAUSE = 72,
    F1 = 58,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    LEFT_SHIFT = 225,
    LEFT_CONTROL = 224,
    LEFT_ALT = 226,
    LEFT_SUPER = 227,
    RIGHT_SHIFT = 229,
    RIGHT_CONTROL = 228,
    RIGHT_ALT = 230,
    RIGHT_SUPER = 231,
    MENU = 118,
    UNKNOWN = 0
};

/// @brief Represents sdl mouse button IDs.
/// @details There is a subset of sdl mouse button IDs.
enum class Mousecode : int {
    BUTTON_1 = 1,  // Left mouse button
    BUTTON_2 = 2,  // Right mouse button
    BUTTON_3 = 3,  // Middle mouse button
    BUTTON_4 = 4,  // Side mouse button 1
    BUTTON_5 = 5,  // Side mouse button 2
    UNKNOWN = 0,
};

inline Mousecode mousecodes_all[] {
    Mousecode::BUTTON_1,
    Mousecode::BUTTON_2,
    Mousecode::BUTTON_3,
    Mousecode::BUTTON_4,
    Mousecode::BUTTON_5,
};

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

    virtual void pollEvents(bool waitForRefresh) = 0;

    virtual const char* getClipboardText() const = 0;
    virtual void setClipboardText(const char* str) = 0;

    virtual void startTextInput() = 0;
    virtual void stopTextInput() = 0;

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

    virtual ObserverHandler addKeyCallback(
        Keycode key, KeyCallback callback
    ) = 0;

    virtual const std::vector<Keycode>& getPressedKeys() const = 0;
    virtual const std::vector<std::uint32_t>& getCodepoints() const = 0;

    ObserverHandler addCallback(const std::string& name, KeyCallback callback) {
        return getBindings().require(name).onactived.add(callback);
    }
};
