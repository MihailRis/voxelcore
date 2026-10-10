#pragma once

#include "window/wayland/WaylandCommon.hpp"

#include "window/detail/BaseInput.hpp"

#include "window/Window.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <optional>
#include <string>
#include <thread>
#include <vector>

/// @brief Offset between the evdev keycode and the xkb one
static constexpr uint32_t XKB_KEYCODE_OFFSET = 8;

class WaylandInput : public BaseInput {
public:
    void pollEvents(bool waitForRefresh) override;

    const char* getClipboardText() const override {
        return clipboard.c_str();
    }

    void requestClipboardText() {
        if (state.dataDevice == nullptr || selection == nullptr) {
            return;
        }
        const char* mime = pickMime();
        if (mime == nullptr) {
            clipboard.clear();
            return;
        }
        int fds[2];
        if (pipe(fds) == -1) {
            return;
        }
        if (clipboardFd >= 0) {
            close(clipboardFd);
        }
        clipboard.clear();
        clipboardFd = fds[0];
        fcntl(clipboardFd, F_SETFL, O_NONBLOCK);
        wl_data_offer_receive(selection, mime, fds[1]);
        close(fds[1]);
        if (state.display) {
            wl_display_flush(state.display);
        }
    }

    void updateClipboard() {
        if (clipboardFd < 0) {
            return;
        }
        char buffer[4096];
        const ssize_t length = read(clipboardFd, buffer, sizeof(buffer));
        if (length > 0) {
            clipboard.append(buffer, length);
            return;
        }
        if (length == 0) {
            close(clipboardFd);
            clipboardFd = -1;
        }
    }

    void setClipboardText(const char* text) override {
        if (state.dataDeviceManager == nullptr || state.dataDevice == nullptr) {
            return;
        }
        auto* source = wl_data_device_manager_create_data_source(
            state.dataDeviceManager
        );
        wl_data_source_add_listener(source, &sourceListener(), nullptr);
        wl_data_source_offer(source, "text/plain;charset=utf-8");
        wl_data_source_offer(source, "text/plain");
        wl_data_source_offer(source, "UTF8_STRING");
        sourceText = text ? text : "";
        clipboard = sourceText;
        if (clipboardFd >= 0) {
            close(clipboardFd);
            clipboardFd = -1;
        }
        wl_data_device_set_selection(
            state.dataDevice, source, state.inputSerial
        );
        wl_display_flush(state.display);
    }

    void toggleCursor() override {
        cursorDrag = false;
        if (cursorLocked) {
            if (state.lockedPointer) {
                zwp_locked_pointer_v1_destroy(state.lockedPointer);
                state.lockedPointer = nullptr;
            }
            if (state.relativePointer) {
                zwp_relative_pointer_v1_destroy(state.relativePointer);
                state.relativePointer = nullptr;
            }
            cursorLocked = false;
            applyCursor();
            return;
        }
        if (state.pointerConstraints && state.pointer && state.surface) {
            state.lockedPointer = zwp_pointer_constraints_v1_lock_pointer(
                state.pointerConstraints,
                state.surface,
                state.pointer,
                nullptr,
                ZWP_POINTER_CONSTRAINTS_V1_LIFETIME_PERSISTENT
            );
        }
        if (state.relativePointerManager && state.pointer) {
            state.relativePointer =
                zwp_relative_pointer_manager_v1_get_relative_pointer(
                    state.relativePointerManager, state.pointer
                );
            zwp_relative_pointer_v1_add_listener(
                state.relativePointer, &relativePointerListener(), nullptr
            );
        }
        if (state.pointer && state.entered) {
            wl_pointer_set_cursor(
                state.pointer, state.pointerSerial, nullptr, 0, 0
            );
        }
        cursorLocked = true;
    }

    void setCursorShape(CursorShape shape) {
        if (cursor == shape) {
            return;
        }
        cursor = shape;
        applyCursor();
    }

    void applyCursor() {
        if (cursorLocked || !state.entered || !state.cursorShapeDevice) {
            return;
        }
        wp_cursor_shape_device_v1_set_shape(
            state.cursorShapeDevice, state.pointerSerial,
            shapeId(overrideCursor ? *overrideCursor : cursor)
        );
    }

    void setOverrideCursor(std::optional<CursorShape> shape) {
        if (overrideCursor == shape) {
            return;
        }
        overrideCursor = shape;
        applyCursor();
    }

    /// @brief Release every pressed key, e.g. when the keyboard focus is lost
    void releasePressedKeys() {
        const auto keys = pressedKeys;
        for (auto key : keys) {
            onKeyCallback(static_cast<int>(key), false);
        }
        repeatKeycode = 0;
    }

    /// @brief Forget the cached clipboard text
    void clearClipboardText() {
        clipboard.clear();
    }

    void onRelativeMotion(double dx, double dy) {
        delta.x += dx;
        delta.y += dy;
    }

    void onKey(uint32_t key, bool pressed) {
        if (state.xkbState == nullptr || state.xkbKeymap == nullptr) {
            return;
        }
        xkb_keycode_t code = key + XKB_KEYCODE_OFFSET;
        const xkb_keysym_t* syms = nullptr;
        if (xkb_keymap_key_get_syms_by_level(
                state.xkbKeymap, code, 0, 0, &syms
            ) > 0) {
            int keycode = keycode_from_keysym(syms[0]);
            if (keycode != 0) {
                onKeyCallback(keycode, pressed);
            }
            if (pressed) {
                repeatKeycode = code;
                repeatTime = now();
                lastRepeat = repeatTime;
            } else if (code == repeatKeycode) {
                repeatKeycode = 0;
            }
        }
        if (pressed) {
            appendCodepoints(code);
        }
        xkb_state_update_key(
            state.xkbState, code, pressed ? XKB_KEY_DOWN : XKB_KEY_UP
        );
    }

    void refreshWindow();

    wl_data_offer* selection = nullptr;
    std::vector<std::string> selectionMimes;
    std::vector<std::string> pendingMimes;
    std::string clipboard;
    int clipboardFd = -1;
    std::string sourceText;
private:
    static uint32_t shapeId(CursorShape shape) {
        switch (shape) {
            case CursorShape::ARROW:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_DEFAULT;
            case CursorShape::TEXT:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_TEXT;
            case CursorShape::CROSSHAIR:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_CROSSHAIR;
            case CursorShape::POINTER:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_POINTER;
            case CursorShape::EW_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_EW_RESIZE;
            case CursorShape::NS_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NS_RESIZE;
            case CursorShape::NWSE_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NWSE_RESIZE;
            case CursorShape::NESW_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NESW_RESIZE;
            case CursorShape::ALL_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_ALL_RESIZE;
            case CursorShape::N_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_N_RESIZE;
            case CursorShape::S_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_S_RESIZE;
            case CursorShape::E_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_E_RESIZE;
            case CursorShape::W_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_W_RESIZE;
            case CursorShape::NE_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NE_RESIZE;
            case CursorShape::NW_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NW_RESIZE;
            case CursorShape::SE_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_SE_RESIZE;
            case CursorShape::SW_RESIZE:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_SW_RESIZE;
            case CursorShape::NOT_ALLOWED:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NOT_ALLOWED;
        }
        return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_DEFAULT;
    }

    static void dispatch(bool block) {
        dispatch_events(block ? 500 : 0);
    }

    static double now() {
        return waylandNow();
    }

    void appendCodepoints(xkb_keycode_t code) {
        char buffer[64];
        int length = xkb_state_key_get_utf8(
            state.xkbState, code, buffer, sizeof(buffer)
        );
        if (length <= 0) {
            return;
        }
        const auto* bytes = reinterpret_cast<const ubyte*>(buffer);
        uint codepoint = 0;
        for (int i = 0; i < length; i++) {
            if ((bytes[i] & 0xC0) == 0x80) {
                codepoint = (codepoint << 6) | (bytes[i] & 0x3F);
                continue;
            }
            if (codepoint >= 0x20 && codepoint != 0x7F) {
                codepoints.push_back(codepoint);
            }
            if ((bytes[i] & 0x80) == 0) {
                codepoint = bytes[i];
            } else if ((bytes[i] & 0xE0) == 0xC0) {
                codepoint = bytes[i] & 0x1F;
            } else if ((bytes[i] & 0xF0) == 0xE0) {
                codepoint = bytes[i] & 0x0F;
            } else {
                codepoint = bytes[i] & 0x07;
            }
        }
        if (codepoint >= 0x20 && codepoint != 0x7F) {
            codepoints.push_back(codepoint);
        }
    }

    void updateRepeat() {
        if (repeatKeycode == 0 || state.repeatRate == 0 || !state.xkbState) {
            return;
        }
        int keycode = 0;
        const xkb_keysym_t* syms = nullptr;
        if (xkb_keymap_key_get_syms_by_level(
                state.xkbKeymap, repeatKeycode, 0, 0, &syms
            ) > 0) {
            keycode = keycode_from_keysym(syms[0]);
        }
        if (keycode == 0 || !pressed(static_cast<Keycode>(keycode))) {
            return;
        }
        double time = now();
        if (time - repeatTime < state.repeatDelay / 1000.0) {
            return;
        }
        double interval = 1.0 / std::max<uint32_t>(state.repeatRate, 1);
        if (time - lastRepeat < interval) {
            return;
        }
        lastRepeat = time;
        onKeyCallback(keycode, true);
        appendCodepoints(repeatKeycode);
    }

    const char* pickMime() const {
        for (const auto& mime : selectionMimes) {
            if (mime == "text/plain;charset=utf-8" || mime == "text/plain" ||
                mime == "UTF8_STRING") {
                return mime.c_str();
            }
        }
        return nullptr;
    }

    static const wl_data_source_listener& sourceListener() {
        static wl_data_source_listener listener {};
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            listener.send = [] (
                void*, wl_data_source*, const char*, int32_t fd
            ) {
                if (input) {
                    const std::string& text = input->sourceText;
                    if (!text.empty() &&
                        write(fd, text.data(), text.size()) < 0) {
                        waylandLogger.warning()
                            << "could not write to the clipboard";
                    }
                }
                close(fd);
            };
            listener.cancelled = [] (void*, wl_data_source* source) {
                wl_data_source_destroy(source);
            };
            listener.target = ignore_event<>;
            listener.dnd_drop_performed = ignore_event<>;
            listener.dnd_finished = ignore_event<>;
            listener.action = ignore_event<>;
        }
        return listener;
    }

    static const zwp_relative_pointer_v1_listener& relativePointerListener() {
        static zwp_relative_pointer_v1_listener listener {};
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            listener.relative_motion = [] (
                void*, zwp_relative_pointer_v1*, uint32_t, uint32_t,
                wl_fixed_t dx, wl_fixed_t dy, wl_fixed_t, wl_fixed_t
            ) {
                if (input) {
                    input->onRelativeMotion(
                        wl_fixed_to_double(dx), wl_fixed_to_double(dy)
                    );
                }
            };
        }
        return listener;
    }

    CursorShape cursor = CursorShape::ARROW;
    std::optional<CursorShape> overrideCursor;
    xkb_keycode_t repeatKeycode = 0;
    double repeatTime = 0.0;
    double lastRepeat = 0.0;
};
