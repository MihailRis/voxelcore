#pragma once

#include "window/input.hpp"

#include <glm/vec2.hpp>
#include <unordered_map>
#include <vector>

inline constexpr short KEYS_BUFFER_SIZE = 1036;
inline constexpr short MOUSE_KEYS_OFFSET = 1024;

class BaseInput : public Input {
public:
    std::vector<uint> codepoints;
    int scroll = 0;

    void onKeyCallback(int key, bool pressed);
    void onMouseCallback(int button, bool pressed);

    bool isCursorLocked() const override;
    void setCursorPosition(double xpos, double ypos);

    Bindings& getBindings() override;
    const Bindings& getBindings() const override;

    ObserverHandler addKeyCallback(Keycode key, InputCallback callback) override;
    ObserverHandler addMouseCallback(Mousecode button, InputCallback callback) override;

    const std::vector<Keycode>& getPressedKeys() const override;
    const std::vector<uint>& getCodepoints() const override;

    CursorState getCursor() const override;
    int getScroll() override;

    bool pressed(Keycode key) const override;
    bool jpressed(Keycode keycode) const override;
    bool clicked(Mousecode code) const override;
    bool jclicked(Mousecode code) const override;

    void simulateKey(Keycode key, bool pressed) override;
    void simulateClick(int button, bool pressed) override;
    void simulateCursorPos(double xpos, double ypos) override;
    void simulateCodepoint(uint codepoint) override;
protected:
    void beginFrame();
    void updateBindings();

    uint currentFrame = 0;
    uint frames[KEYS_BUFFER_SIZE] {};
    std::vector<Keycode> pressedKeys;
    Bindings bindings;
    bool keys[KEYS_BUFFER_SIZE] {};
    std::unordered_map<Keycode, util::HandlersList<>> keyCallbacks;
    bool cursorLocked = false;
    bool cursorDrag = false;
    glm::vec2 delta {};
    glm::vec2 cursor {};
};
