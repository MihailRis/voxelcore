#include "window/wayland/WaylandWindow.hpp"
#include "window/wayland/WaylandInput.hpp"

#include <sys/mman.h>

static void pointer_enter(
    void*, wl_pointer*, uint32_t serial, wl_surface* surface, wl_fixed_t sx,
    wl_fixed_t sy
) {
    state.pointerSerial = serial;
    state.entered = surface == state.surface;
    const double x = wl_fixed_to_double(sx);
    const double y = wl_fixed_to_double(sy);
    if (state.entered && window && window->handleBarMotion(x, y)) {
        return;
    }
    if (input) {
        input->setCursorPosition(
            window ? window->contentX(x) : x, window ? window->contentY(y) : y
        );
        if (state.entered) {
            input->applyCursor();
        }
        input->refreshWindow();
    }
}

static void pointer_leave(void*, wl_pointer*, uint32_t, wl_surface*) {
    state.entered = false;
    if (window) {
        window->onPointerLeave();
    }
}

static void pointer_motion(
    void*, wl_pointer*, uint32_t, wl_fixed_t sx, wl_fixed_t sy
) {
    const double x = wl_fixed_to_double(sx);
    const double y = wl_fixed_to_double(sy);
    if (window && window->handleBarMotion(x, y)) {
        return;
    }
    if (input) {
        input->setCursorPosition(
            window ? window->contentX(x) : x, window ? window->contentY(y) : y
        );
        input->refreshWindow();
    }
}

static void pointer_button(
    void*, wl_pointer*, uint32_t serial, uint32_t, uint32_t button,
    uint32_t buttonState
) {
    state.inputSerial = serial;
    // BTN_LEFT .. BTN_TASK from linux/input-event-codes.h
    constexpr uint32_t EVDEV_BUTTON_FIRST = 0x110;
    constexpr uint32_t EVDEV_BUTTON_LAST = 0x117;
    int index = button >= EVDEV_BUTTON_FIRST && button <= EVDEV_BUTTON_LAST
        ? static_cast<int>(button - EVDEV_BUTTON_FIRST)
        : -1;
    const bool pressed = buttonState == WL_POINTER_BUTTON_STATE_PRESSED;
    if (index >= 0 && window &&
        window->handleBarButton(serial, index, pressed)) {
        return;
    }
    if (index >= 0 && input) {
        input->onMouseCallback(index, pressed);
        input->refreshWindow();
    }
}

static void pointer_axis(
    void*, wl_pointer*, uint32_t, uint32_t axis, wl_fixed_t value
) {
    if (input && axis == WL_POINTER_AXIS_VERTICAL_SCROLL) {
        if (window && window->handleBarScroll()) {
            return;
        }
        input->scroll += wl_fixed_to_double(value) > 0 ? -1 : 1;
        input->refreshWindow();
    }
}

static void keyboard_keymap(
    void*, wl_keyboard*, uint32_t format, int32_t fd, uint32_t size
) {
    if (format != WL_KEYBOARD_KEYMAP_FORMAT_XKB_V1) {
        close(fd);
        return;
    }
    char* data = static_cast<char*>(
        mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0)
    );
    close(fd);
    if (data == MAP_FAILED) {
        return;
    }
    if (state.xkbContext == nullptr) {
        state.xkbContext = xkb_context_new(XKB_CONTEXT_NO_FLAGS);
    }
    xkb_keymap* keymap = xkb_keymap_new_from_string(
        state.xkbContext, data, XKB_KEYMAP_FORMAT_TEXT_V1,
        XKB_KEYMAP_COMPILE_NO_FLAGS
    );
    munmap(data, size);
    if (keymap == nullptr) {
        waylandLogger.error() << "could not parse the keyboard keymap";
        return;
    }
    if (state.xkbState) {
        xkb_state_unref(state.xkbState);
    }
    if (state.xkbKeymap) {
        xkb_keymap_unref(state.xkbKeymap);
    }
    state.xkbKeymap = keymap;
    state.xkbState = xkb_state_new(keymap);
}

static void keyboard_enter(
    void*, wl_keyboard*, uint32_t serial, wl_surface*, wl_array*
) {
    state.inputSerial = serial;
}

static void keyboard_leave(void*, wl_keyboard*, uint32_t, wl_surface*) {
    if (input) {
        input->releasePressedKeys();
    }
}

static void keyboard_key(
    void*, wl_keyboard*, uint32_t serial, uint32_t, uint32_t key,
    uint32_t keyState
) {
    state.inputSerial = serial;
    if (input) {
        input->onKey(key, keyState == WL_KEYBOARD_KEY_STATE_PRESSED);
        input->refreshWindow();
    }
}

static void keyboard_modifiers(
    void*, wl_keyboard*, uint32_t serial, uint32_t depressed, uint32_t latched,
    uint32_t locked, uint32_t group
) {
    state.inputSerial = serial;
    if (state.xkbState) {
        xkb_state_update_mask(
            state.xkbState, depressed, latched, locked, 0, 0, group
        );
    }
}

static void keyboard_repeat_info(
    void*, wl_keyboard*, int32_t rate, int32_t delay
) {
    state.repeatRate = rate > 0 ? static_cast<uint32_t>(rate) : 0;
    state.repeatDelay = delay > 0 ? delay : 400;
}

static void seat_capabilities(void*, wl_seat* seat, uint32_t capabilities) {
    static const wl_pointer_listener pointerListener = [] {
        wl_pointer_listener listener {};
        listener.enter = pointer_enter;
        listener.leave = pointer_leave;
        listener.motion = pointer_motion;
        listener.button = pointer_button;
        listener.axis = pointer_axis;
        listener.frame = ignore_event<>;
        listener.axis_source = ignore_event<>;
        listener.axis_stop = ignore_event<>;
        listener.axis_discrete = ignore_event<>;
        return listener;
    }();

    static const wl_keyboard_listener keyboardListener = [] {
        wl_keyboard_listener listener {};
        listener.keymap = keyboard_keymap;
        listener.enter = keyboard_enter;
        listener.leave = keyboard_leave;
        listener.key = keyboard_key;
        listener.modifiers = keyboard_modifiers;
        listener.repeat_info = keyboard_repeat_info;
        return listener;
    }();

    bool hasPointer = capabilities & WL_SEAT_CAPABILITY_POINTER;
    bool hasKeyboard = capabilities & WL_SEAT_CAPABILITY_KEYBOARD;
    if (hasPointer && state.pointer == nullptr) {
        state.pointer = wl_seat_get_pointer(seat);
        wl_pointer_add_listener(state.pointer, &pointerListener, nullptr);
        if (state.cursorShapeManager) {
            state.cursorShapeDevice = wp_cursor_shape_manager_v1_get_pointer(
                state.cursorShapeManager, state.pointer
            );
        }
    } else if (!hasPointer && state.pointer) {
        if (state.cursorShapeDevice) {
            wp_cursor_shape_device_v1_destroy(state.cursorShapeDevice);
            state.cursorShapeDevice = nullptr;
        }
        wl_pointer_destroy(state.pointer);
        state.pointer = nullptr;
    }
    if (hasKeyboard && state.keyboard == nullptr) {
        state.keyboard = wl_seat_get_keyboard(seat);
        wl_keyboard_add_listener(state.keyboard, &keyboardListener, nullptr);
    } else if (!hasKeyboard && state.keyboard) {
        wl_keyboard_destroy(state.keyboard);
        state.keyboard = nullptr;
    }
}

static void seat_name(void*, wl_seat*, const char*) {
}

static void wm_base_ping(void*, xdg_wm_base* base, uint32_t serial) {
    xdg_wm_base_pong(base, serial);
}

void toplevel_configure(
    void*, xdg_toplevel*, int32_t width, int32_t height, wl_array* states
) {
    if (window) {
        window->onConfigure(width, height, states);
    }
}

void toplevel_close(void*, xdg_toplevel*) {
    if (window) {
        window->setShouldClose(true);
    }
}

void toplevel_configure_bounds(void*, xdg_toplevel*, int32_t, int32_t) {
}

void toplevel_wm_capabilities(void*, xdg_toplevel*, wl_array*) {
}

void xdg_surface_configure(void*, xdg_surface* surface, uint32_t serial) {
    xdg_surface_ack_configure(surface, serial);
    if (window) {
        window->applyConfigure();
    }
}

static void data_offer_offer(void*, wl_data_offer*, const char* mime) {
    if (input) {
        input->pendingMimes.push_back(mime);
    }
}

void data_device_data_offer(void*, wl_data_device*, wl_data_offer* offer) {
    static const wl_data_offer_listener listener = [] {
        wl_data_offer_listener value {};
        value.offer = data_offer_offer;
        value.source_actions = ignore_event<>;
        value.action = ignore_event<>;
        return value;
    }();
    wl_data_offer_add_listener(offer, &listener, nullptr);
}

void data_device_selection(void*, wl_data_device*, wl_data_offer* offer) {
    if (input == nullptr) {
        return;
    }
    if (input->selection) {
        wl_data_offer_destroy(input->selection);
    }
    input->selection = offer;
    input->pendingMimes.clear();
    if (offer == nullptr) {
        input->selectionMimes.clear();
        input->clearClipboardText();
        return;
    }
    input->selectionMimes = std::move(input->pendingMimes);
    input->requestClipboardText();
}

void registry_global(
    void*, wl_registry* registry, uint32_t name, const char* interface,
    uint32_t version
) {
    if (strcmp(interface, wl_compositor_interface.name) == 0) {
        state.compositor = static_cast<wl_compositor*>(wl_registry_bind(
            registry, name, &wl_compositor_interface, std::min(version, 4u)
        ));
    } else if (strcmp(interface, xdg_wm_base_interface.name) == 0) {
        state.wmBase = static_cast<xdg_wm_base*>(wl_registry_bind(
            registry, name, &xdg_wm_base_interface, std::min(version, 6u)
        ));
        static const xdg_wm_base_listener listener = [] {
            xdg_wm_base_listener value {};
            value.ping = wm_base_ping;
            return value;
        }();
        xdg_wm_base_add_listener(state.wmBase, &listener, nullptr);
    } else if (strcmp(interface, wl_seat_interface.name) == 0) {
        // A pointer inherits the seat version, so the pointer listener only
        // has to cover the events of version 7 and below. Version 8 adds
        // axis_value120 and version 9 axis_relative_direction, whose members
        // do not exist in the wayland 1.20 headers either.
        state.seat = static_cast<wl_seat*>(wl_registry_bind(
            registry, name, &wl_seat_interface, std::min(version, 7u)
        ));
        static const wl_seat_listener listener = [] {
            wl_seat_listener value {};
            value.capabilities = seat_capabilities;
            value.name = seat_name;
            return value;
        }();
        wl_seat_add_listener(state.seat, &listener, nullptr);
    } else if (strcmp(interface, wl_data_device_manager_interface.name) == 0) {
        state.dataDeviceManager = static_cast<wl_data_device_manager*>(
            wl_registry_bind(
                registry, name, &wl_data_device_manager_interface,
                std::min(version, 3u)
            )
        );
    } else if (
        strcmp(interface, wp_cursor_shape_manager_v1_interface.name) == 0
    ) {
        state.cursorShapeManager = static_cast<wp_cursor_shape_manager_v1*>(
            wl_registry_bind(
                registry, name, &wp_cursor_shape_manager_v1_interface,
                std::min(version, 2u)
            )
        );
    } else if (
        strcmp(interface, zwp_pointer_constraints_v1_interface.name) == 0
    ) {
        state.pointerConstraints = static_cast<zwp_pointer_constraints_v1*>(
            wl_registry_bind(
                registry, name, &zwp_pointer_constraints_v1_interface, 1
            )
        );
    } else if (
        strcmp(interface, zwp_relative_pointer_manager_v1_interface.name) == 0
    ) {
        state.relativePointerManager =
            static_cast<zwp_relative_pointer_manager_v1*>(wl_registry_bind(
                registry, name, &zwp_relative_pointer_manager_v1_interface, 1
            ));
    } else if (
        strcmp(interface, zwp_idle_inhibit_manager_v1_interface.name) == 0
    ) {
        state.idleInhibitManager = static_cast<zwp_idle_inhibit_manager_v1*>(
            wl_registry_bind(
                registry, name, &zwp_idle_inhibit_manager_v1_interface, 1
            )
        );
    } else if (
        strcmp(interface, zxdg_decoration_manager_v1_interface.name) == 0
    ) {
        state.decorationManager = static_cast<zxdg_decoration_manager_v1*>(
            wl_registry_bind(
                registry, name, &zxdg_decoration_manager_v1_interface, 1
            )
        );
    }
}

void registry_global_remove(void*, wl_registry*, uint32_t) {
}

void zxdg_decoration_configure(
    void*, zxdg_toplevel_decoration_v1*, uint32_t mode
) {
    serverDecorations = mode == ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE;
    if (window != nullptr) {
        if (serverDecorations) {
            window->disableOwnDecorations();
        } else {
            window->enableOwnDecorations();
        }
    }
}

