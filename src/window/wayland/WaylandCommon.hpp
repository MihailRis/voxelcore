#pragma once

#include <GL/glew.h>

#include "debug/Logger.hpp"

#include <GLFW/glfw3.h>
#include <wayland-client.h>
#include <xkbcommon/xkbcommon.h>

#include "cursor-shape-v1-client-protocol.h"
#include "idle-inhibit-unstable-v1-client-protocol.h"
#include "pointer-constraints-unstable-v1-client-protocol.h"
#include "relative-pointer-unstable-v1-client-protocol.h"
#include "xdg-decoration-unstable-v1-client-protocol.h"
#include "xdg-shell-client-protocol.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <wayland-egl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <poll.h>
#include <stack>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

const wl_registry_listener& registry_listener();
const wl_data_device_listener& data_device_listener();
const xdg_surface_listener& surface_listener();
const xdg_toplevel_listener& toplevel_listener();

struct GlStateGuard {
    GLint values[16] {};
    GLint attribs[3][7] {};
    GLint viewport[4] {};
    GLint colorMask[4] {};
    GLboolean flags[6] {};
    GLboolean attribFlags[3] {};
    GlStateGuard();
    ~GlStateGuard();
};

GLuint compile_program(const char* vertexSource, const char* fragmentSource);

extern debug::Logger waylandLogger;

struct WaylandState {
    wl_display* display = nullptr;
    wl_registry* registry = nullptr;
    wl_compositor* compositor = nullptr;
    wl_surface* surface = nullptr;
    xdg_wm_base* wmBase = nullptr;
    xdg_surface* xdgSurface = nullptr;
    xdg_toplevel* toplevel = nullptr;
    zxdg_toplevel_decoration_v1* decoration = nullptr;
    wl_seat* seat = nullptr;
    wl_pointer* pointer = nullptr;
    wl_keyboard* keyboard = nullptr;
    wl_data_device_manager* dataDeviceManager = nullptr;
    wl_data_device* dataDevice = nullptr;
    wp_cursor_shape_manager_v1* cursorShapeManager = nullptr;
    wp_cursor_shape_device_v1* cursorShapeDevice = nullptr;
    zwp_pointer_constraints_v1* pointerConstraints = nullptr;
    zwp_relative_pointer_manager_v1* relativePointerManager = nullptr;
    zwp_relative_pointer_v1* relativePointer = nullptr;
    zwp_locked_pointer_v1* lockedPointer = nullptr;
    zwp_idle_inhibit_manager_v1* idleInhibitManager = nullptr;
    zwp_idle_inhibitor_v1* idleInhibitor = nullptr;
    zxdg_decoration_manager_v1* decorationManager = nullptr;
    xkb_context* xkbContext = nullptr;
    xkb_keymap* xkbKeymap = nullptr;
    xkb_state* xkbState = nullptr;
    uint32_t pointerSerial = 0;
    uint32_t inputSerial = 0;
    uint32_t repeatRate = 25;
    uint32_t repeatDelay = 400;
    bool entered = false;
};

extern WaylandState state;

class WaylandInput;
class WaylandWindow;

extern WaylandInput* input;
extern WaylandWindow* window;

template <typename... Args>
void ignore_event(Args...) {
}

bool decorations_enabled();
void dispatch_events(int timeoutMs);
int keycode_from_keysym(xkb_keysym_t sym);

extern bool serverDecorations;

void zxdg_decoration_configure(
    void*, zxdg_toplevel_decoration_v1*, uint32_t mode
);
void on_display_error();
