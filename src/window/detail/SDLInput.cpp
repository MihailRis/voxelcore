#include "window/detail/SDLInput.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_clipboard.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_keyboard.h>
#include <SDL3/SDL_keycode.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_scancode.h>
#include <SDL3/SDL_video.h>

#include <string>

#include "debug/Logger.hpp"
#include "util/stringutil.hpp"
#include "window/detail/SDLWindow.hpp"
#include "window/input.hpp"

static debug::Logger logger("input");

static std::unordered_map<std::string, std::uint32_t> keycodes {
    {"enter", SDL_SCANCODE_RETURN},
    {"space", SDL_SCANCODE_SPACE},
    {"backspace", SDL_SCANCODE_BACKSPACE},
    {"caps-lock", SDL_SCANCODE_CAPSLOCK},
    {"escape", SDL_SCANCODE_ESCAPE},
    {"delete", SDL_SCANCODE_DELETE},
    {"home", SDL_SCANCODE_HOME},
    {"end", SDL_SCANCODE_END},
    {"tab", SDL_SCANCODE_TAB},
    {"insert", SDL_SCANCODE_INSERT},
    {"page-down", SDL_SCANCODE_PAGEDOWN},
    {"page-up", SDL_SCANCODE_PAGEUP},
    {"left-shift", SDL_SCANCODE_LSHIFT},
    {"right-shift", SDL_SCANCODE_RSHIFT},
    {"left-ctrl", SDL_SCANCODE_LCTRL},
    {"right-ctrl", SDL_SCANCODE_RCTRL},
    {"left-alt", SDL_SCANCODE_LALT},
    {"right-alt", SDL_SCANCODE_RALT},
    {"left-super", SDL_SCANCODE_LGUI},
    {"right-super", SDL_SCANCODE_RGUI},
    {"grave-accent", SDL_SCANCODE_GRAVE},
    {"left", SDL_SCANCODE_LEFT},
    {"right", SDL_SCANCODE_RIGHT},
    {"down", SDL_SCANCODE_DOWN},
    {"up", SDL_SCANCODE_UP},
};
static std::unordered_map<std::string, std::uint32_t> mousecodes {
    {"left", SDL_BUTTON_LEFT},
    {"right", SDL_BUTTON_RIGHT},
    {"middle", SDL_BUTTON_MIDDLE},
    {"side1", SDL_BUTTON_X1},
    {"side2", SDL_BUTTON_X2},
};

static std::unordered_map<int, std::string> keynames {};
static std::unordered_map<int, std::string> buttonsnames {};

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

void input_util::initialize() {
    keycodes[std::to_string(0)] = SDL_SCANCODE_0;
    for (int i = 1; i <= 9; i++) {
        keycodes[std::to_string(i)] = SDL_SCANCODE_1 + i;
    }
    for (int i = 0; i < 25; i++) {
        keycodes["f" + std::to_string(i + 1)] = SDL_SCANCODE_F1 + i;
    }
    for (char i = 'a'; i <= 'z'; i++) {
        keycodes[std::string({i})] = SDL_SCANCODE_A - 'a' + i;
    }
    for (const auto& entry : keycodes) {
        keynames[entry.second] = entry.first;
    }
    for (const auto& entry : mousecodes) {
        buttonsnames[entry.second] = entry.first;
    }
}

std::string input_util::to_string(Keycode code) {
    auto icode_repr = static_cast<SDL_Scancode>(code);
    const char* name = SDL_GetKeyName(
        SDL_GetKeyFromScancode(icode_repr, SDL_KMOD_NONE, false)
    );
    return std::string(name);
}

Keycode input_util::keycode_from(const std::string& name) {
    auto found = std::find_if(
        std::begin(keynames), std::end(keynames), [&name](auto&& p) {
            return p.second == name;
        }
    );
    // Compatibility with old names
    if (found != keynames.end()) return static_cast<Keycode>(found->first);

    return static_cast<Keycode>(SDL_GetScancodeFromName(name.c_str()));
}

Mousecode input_util::mousecode_from(const std::string& name) {
    const auto& found = mousecodes.find(name);
    if (found == mousecodes.end()) {
        return Mousecode::UNKNOWN;
    }
    return static_cast<Mousecode>(found->second);
}

SDLInput::SDLInput(SDLWindow& window) : window(window) {
    input_util::initialize();
}

void SDLInput::pollEvents() {
    delta.x = 0.0f;
    delta.y = 0.0f;
    scroll = 0;
    currentFrame++;
    codepoints.clear();
    pressedKeys.clear();

    std::string text {};
    uint size {};

    bool prevPressed = false;

    static SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_EVENT_QUIT:
                window.setShouldClose(true);
                break;
            case SDL_EVENT_KEY_DOWN:
                prevPressed = keys[event.key.scancode];
                keys[event.key.scancode] = true;
                frames[event.key.scancode] = currentFrame;
                if (!prevPressed) {
                    keyCallbacks[static_cast<Keycode>(event.key.scancode)]
                        .notify();
                }
                pressedKeys.push_back(static_cast<Keycode>(event.key.scancode));
                break;
            case SDL_EVENT_KEY_UP:
                keys[event.key.scancode] = false;
                frames[event.key.scancode] = currentFrame;
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                prevPressed = keys[event.button.button + mouse_keys_offset];
                keys[event.button.button + mouse_keys_offset] = true;
                frames[event.button.button + mouse_keys_offset] = currentFrame;
                if (!prevPressed) {
                    keyCallbacks[static_cast<Keycode>(
                                     event.button.button + mouse_keys_offset
                                 )]
                        .notify();
                }
                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                keys[event.button.button + mouse_keys_offset] = false;
                frames[event.button.button + mouse_keys_offset] = currentFrame;
                break;
            case SDL_EVENT_MOUSE_MOTION:
                if (cursorDrag) {
                    delta += glm::vec2 {event.motion.xrel, event.motion.yrel};
                } else {
                    cursorDrag = true;
                }
                cursor = {event.motion.x, event.motion.y};
                break;
            case SDL_EVENT_MOUSE_WHEEL:
                scroll += event.wheel.integer_y;
                break;
            case SDL_EVENT_WINDOW_RESIZED:
                window.setSize({event.window.data1, event.window.data2});
                break;
            case SDL_EVENT_TEXT_INPUT:
                text += event.text.text;
        }
    }
    if (!text.empty()) {
        codepoints.push_back(util::decode_utf8(size, text.c_str()));
    }

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

const char* SDLInput::getClipboardText() const {
    return SDL_GetClipboardText();
}

void SDLInput::setClipboardText(const char* text) {
    SDL_SetClipboardText(text);
}

void SDLInput::startTextInput() {
    SDL_StartTextInput(window.getSdlWindow());
}
void SDLInput::stopTextInput() {
    SDL_StopTextInput(window.getSdlWindow());
}

int SDLInput::getScroll() {
    return scroll;
}

bool SDLInput::pressed(Keycode key) const {
    int keycode = static_cast<int>(key);
    if (keycode < 0 || keycode >= keys_buffer_size) {
        return false;
    }
    return keys[keycode];
}
bool SDLInput::jpressed(Keycode keycode) const {
    return pressed(keycode) &&
           frames[static_cast<int>(keycode)] == currentFrame;
}

bool SDLInput::clicked(Mousecode code) const {
    return pressed(
        static_cast<Keycode>(mouse_keys_offset + static_cast<int>(code))
    );
}
bool SDLInput::jclicked(Mousecode code) const {
    return clicked(code) &&
           frames[static_cast<int>(code) + mouse_keys_offset] == currentFrame;
}

CursorState SDLInput::getCursor() const {
    return {isCursorLocked(), cursor, delta};
}

bool SDLInput::isCursorLocked() const {
    return cursorLocked;
}

void SDLInput::toggleCursor() {
    cursorDrag = false;
    cursorLocked = !cursorLocked;
    SDL_SetWindowRelativeMouseMode(window.getSdlWindow(), cursorLocked);
}

Bindings& SDLInput::getBindings() {
    return bindings;
}

const Bindings& SDLInput::getBindings() const {
    return bindings;
}

ObserverHandler SDLInput::addKeyCallback(Keycode key, KeyCallback callback) {
    return keyCallbacks[key].add(std::move(callback));
}

const std::vector<Keycode>& SDLInput::getPressedKeys() const {
    return pressedKeys;
}

const std::vector<std::uint32_t>& SDLInput::getCodepoints() const {
    return codepoints;
}
