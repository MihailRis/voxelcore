#include "BaseInput.hpp"

void BaseInput::onKeyCallback(int key, bool pressed) {
    bool prevPressed = keys[key];
    keys[key] = pressed;
    frames[key] = currentFrame;
    if (pressed && !prevPressed) {
        const auto& callbacks = keyCallbacks.find(static_cast<Keycode>(key));
        if (callbacks != keyCallbacks.end()) {
            callbacks->second.notify();
        }
    }
    if (pressed && key < MOUSE_KEYS_OFFSET) {
        pressedKeys.push_back(static_cast<Keycode>(key));
    }
}

void BaseInput::onMouseCallback(int button, bool pressed) {
    onKeyCallback(button + MOUSE_KEYS_OFFSET, pressed);
}

bool BaseInput::isCursorLocked() const {
    return cursorLocked;
}

void BaseInput::setCursorPosition(double xpos, double ypos) {
    if (cursorDrag) {
        delta.x += xpos - cursor.x;
        delta.y += ypos - cursor.y;
    } else {
        cursorDrag = true;
    }
    cursor.x = xpos;
    cursor.y = ypos;
}

Bindings& BaseInput::getBindings() {
    return bindings;
}

const Bindings& BaseInput::getBindings() const {
    return bindings;
}

ObserverHandler BaseInput::addKeyCallback(Keycode key, InputCallback callback) {
    return keyCallbacks[key].add(std::move(callback));
}

ObserverHandler BaseInput::addMouseCallback(Mousecode button, InputCallback callback) {
    return addKeyCallback(
        static_cast<Keycode>(MOUSE_KEYS_OFFSET + static_cast<int>(button)),
        std::move(callback)
    );
}

const std::vector<Keycode>& BaseInput::getPressedKeys() const {
    return pressedKeys;
}

const std::vector<uint>& BaseInput::getCodepoints() const {
    return codepoints;
}

CursorState BaseInput::getCursor() const {
    return {isCursorLocked(), cursor, delta};
}

int BaseInput::getScroll() {
    return scroll;
}

bool BaseInput::pressed(Keycode key) const {
    int keycode = static_cast<int>(key);
    if (keycode < 0 || keycode >= KEYS_BUFFER_SIZE) {
        return false;
    }
    return keys[keycode];
}

bool BaseInput::jpressed(Keycode keycode) const {
    return pressed(keycode) &&
           frames[static_cast<int>(keycode)] == currentFrame;
}

bool BaseInput::clicked(Mousecode code) const {
    return pressed(
        static_cast<Keycode>(MOUSE_KEYS_OFFSET + static_cast<int>(code))
    );
}

bool BaseInput::jclicked(Mousecode code) const {
    return clicked(code) &&
           frames[static_cast<int>(code) + MOUSE_KEYS_OFFSET] == currentFrame;
}

void BaseInput::simulateKey(Keycode key, bool pressed) {
    onKeyCallback(static_cast<int>(key), pressed);
}

void BaseInput::simulateClick(int button, bool pressed) {
    onMouseCallback(button, pressed);
}

void BaseInput::simulateCursorPos(double xpos, double ypos) {
    setCursorPosition(xpos, ypos);
}

void BaseInput::simulateCodepoint(uint codepoint) {
    codepoints.push_back(codepoint);
}

void BaseInput::beginFrame() {
    delta.x = 0.0f;
    delta.y = 0.0f;
    scroll = 0;
    currentFrame++;
    codepoints.clear();
    pressedKeys.clear();
}

void BaseInput::updateBindings() {
    for (auto& [_, binding] : bindings.getAll()) {
        if (!binding.enabled) {
            binding.state = false;
            continue;
        }
        binding.justChanged = false;

        bool newstate = false;
        switch (binding.type) {
            case InputType::KEYBOARD:
                newstate = pressed(static_cast<Keycode>(binding.code));
                break;
            case InputType::MOUSE:
                newstate = clicked(static_cast<Mousecode>(binding.code));
                break;
        }

        if (newstate) {
            if (!binding.state) {
                binding.state = true;
                binding.justChanged = true;
                binding.onactived.notify();
            }
        } else {
            if (binding.state) {
                binding.state = false;
                binding.justChanged = true;
            }
        }
    }
}
