#include "window/Window.hpp"
#include "window/detail/BaseInput.hpp"
#include "window/detail/WindowBackends.hpp"

#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"
#include "settings.hpp"
#include "util/platform.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <wayland-client.h>
#include <wayland-egl.h>
#include <xkbcommon/xkbcommon.h>

#include "cursor-shape-v1-client-protocol.h"
#include "idle-inhibit-unstable-v1-client-protocol.h"
#include "pointer-constraints-unstable-v1-client-protocol.h"
#include "relative-pointer-unstable-v1-client-protocol.h"
#include "xdg-decoration-unstable-v1-client-protocol.h"
#include "xdg-shell-client-protocol.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <tuple>
#include <cmath>
#include <cstring>
#include <stack>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

static debug::Logger logger("window");

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

static WaylandState state;

class WaylandInput;
class WaylandWindow;

static WaylandInput* input = nullptr;
static WaylandWindow* window = nullptr;

template <typename... Args>
static void ignore_event(Args...) {
}

static bool decorations_enabled() {
    const char* value = getenv("VOXEL_DECORATIONS");
    return value == nullptr || strcmp(value, "none") != 0;
}

static constexpr int BAR_HEIGHT = 30;
static constexpr int BAR_BUTTON = 30;
static constexpr int BAR_EDGE = 5;
static constexpr int BAR_ICON_SIZE = 16;
static constexpr int BAR_FONT_W = 8;
static constexpr int BAR_FONT_H = 8;
static constexpr int BAR_GLYPHS = 159;

static const uint8_t BAR_FONT[BAR_GLYPHS][BAR_FONT_H] {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x60, 0x60, 0x60, 0x60, 0x00, 0x60, 0x60},
    {0x00, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x14, 0x14, 0x7e, 0x28, 0xfe, 0x28, 0x68},
    {0x00, 0x10, 0x78, 0x70, 0x78, 0x7c, 0x1c, 0x78},
    {0x00, 0x62, 0xf4, 0xfc, 0x6b, 0x1f, 0x17, 0x23},
    {0x00, 0x38, 0x60, 0x30, 0x7a, 0xde, 0x6c, 0x7e},
    {0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x20, 0x60, 0x60, 0x40, 0x40, 0x60, 0x60, 0x20},
    {0x40, 0x60, 0x20, 0x20, 0x20, 0x20, 0x60, 0x40},
    {0x00, 0x20, 0xf0, 0x70, 0xf0, 0x20, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x10, 0x10, 0x7e, 0x10, 0x10},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x40},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60},
    {0x00, 0x20, 0x20, 0x60, 0x40, 0x40, 0xc0, 0x80},
    {0x00, 0x38, 0x6c, 0xcc, 0xcc, 0xcc, 0x6c, 0x38},
    {0x00, 0x70, 0x30, 0x30, 0x30, 0x30, 0x30, 0x7c},
    {0x00, 0x78, 0x18, 0x18, 0x18, 0x30, 0x60, 0x7c},
    {0x00, 0x78, 0x18, 0x18, 0x38, 0x1c, 0x1c, 0xf8},
    {0x00, 0x18, 0x38, 0x78, 0x58, 0xd8, 0xfc, 0x18},
    {0x00, 0x78, 0x40, 0x40, 0x78, 0x0c, 0x0c, 0x78},
    {0x00, 0x38, 0x60, 0x40, 0xf8, 0x6c, 0x6c, 0x38},
    {0x00, 0xfc, 0x18, 0x18, 0x18, 0x30, 0x30, 0x60},
    {0x00, 0x78, 0x6c, 0x6c, 0x78, 0x6c, 0x6c, 0x78},
    {0x00, 0x78, 0xc8, 0xcc, 0x7c, 0x0c, 0x18, 0x70},
    {0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x60, 0x60},
    {0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x60, 0x60},
    {0x00, 0x00, 0x00, 0x06, 0x3c, 0x60, 0x3c, 0x06},
    {0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e, 0x00},
    {0x00, 0x00, 0x00, 0x60, 0x3c, 0x0e, 0x3c, 0x60},
    {0x00, 0x70, 0x18, 0x30, 0x60, 0x00, 0x60, 0x60},
    {0x00, 0x1e, 0x63, 0x5d, 0x55, 0x57, 0x5e, 0x62},
    {0x00, 0x38, 0x38, 0x78, 0x6c, 0x6c, 0xfe, 0xc6},
    {0x00, 0x78, 0x6c, 0x6c, 0x7c, 0x6c, 0x6c, 0x7c},
    {0x00, 0x3c, 0x64, 0xc0, 0xc0, 0xc0, 0x64, 0x3c},
    {0x00, 0x7c, 0x6e, 0x66, 0x66, 0x66, 0x6e, 0x7c},
    {0x00, 0x7c, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7c},
    {0x00, 0x7c, 0x60, 0x60, 0x78, 0x60, 0x60, 0x60},
    {0x00, 0x3c, 0x62, 0xc0, 0xce, 0xc6, 0x66, 0x3e},
    {0x00, 0x66, 0x66, 0x66, 0x7e, 0x66, 0x66, 0x66},
    {0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60},
    {0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60},
    {0x00, 0x6e, 0x7c, 0x78, 0x70, 0x78, 0x6c, 0x66},
    {0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7c},
    {0x00, 0x63, 0x77, 0x77, 0x7f, 0x7f, 0x6b, 0x63},
    {0x00, 0x66, 0x76, 0x76, 0x7e, 0x7e, 0x6e, 0x6e},
    {0x00, 0x3c, 0x66, 0xc6, 0xc6, 0xc6, 0x66, 0x3c},
    {0x00, 0x7c, 0x6c, 0x6c, 0x7c, 0x60, 0x60, 0x60},
    {0x00, 0x3c, 0x66, 0xc6, 0xc6, 0xc6, 0x66, 0x3c},
    {0x00, 0x78, 0x6c, 0x6c, 0x78, 0x6c, 0x6c, 0x66},
    {0x00, 0x78, 0x68, 0x60, 0x78, 0x0c, 0x4c, 0x78},
    {0x00, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30},
    {0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6c, 0x3c},
    {0x00, 0xc6, 0xc6, 0x6c, 0x6c, 0x78, 0x38, 0x38},
    {0x00, 0xcc, 0xcc, 0x7d, 0x7f, 0x77, 0x73, 0x33},
    {0x00, 0xcc, 0x6c, 0x38, 0x38, 0x38, 0x6c, 0xc6},
    {0x00, 0xcc, 0x6c, 0x78, 0x38, 0x30, 0x30, 0x30},
    {0x00, 0xfc, 0x1c, 0x18, 0x30, 0x70, 0x60, 0xfc},
    {0x70, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x70},
    {0x00, 0x80, 0xc0, 0x40, 0x40, 0x60, 0x20, 0x20},
    {0xe0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xe0},
    {0x00, 0x38, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x78, 0x08, 0x78, 0xd8, 0x78},
    {0x40, 0x40, 0x40, 0x78, 0x6c, 0x4c, 0x6c, 0x78},
    {0x00, 0x00, 0x00, 0x78, 0x60, 0xc0, 0x60, 0x78},
    {0x0c, 0x0c, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x7c},
    {0x00, 0x00, 0x00, 0x78, 0xcc, 0xfc, 0xc0, 0x7c},
    {0x70, 0x60, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x60},
    {0x00, 0x00, 0x00, 0x7c, 0xcc, 0xcc, 0xcc, 0x7c},
    {0x40, 0x40, 0x40, 0x78, 0x6c, 0x4c, 0x4c, 0x4c},
    {0x40, 0x40, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40},
    {0x40, 0x40, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40},
    {0x40, 0x40, 0x40, 0x4c, 0x78, 0x70, 0x78, 0x4c},
    {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x00, 0x00, 0x7f, 0x6d, 0x4d, 0x4d, 0x4d},
    {0x00, 0x00, 0x00, 0x78, 0x6c, 0x4c, 0x4c, 0x4c},
    {0x00, 0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0x78},
    {0x00, 0x00, 0x00, 0x78, 0x6c, 0x4c, 0x6c, 0x78},
    {0x00, 0x00, 0x00, 0x7c, 0xcc, 0xcc, 0xcc, 0x7c},
    {0x00, 0x00, 0x00, 0x78, 0x60, 0x40, 0x40, 0x40},
    {0x00, 0x00, 0x00, 0x78, 0xe0, 0x78, 0x18, 0xf8},
    {0x00, 0x00, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x70},
    {0x00, 0x00, 0x00, 0x4c, 0x4c, 0x4c, 0x6c, 0x7c},
    {0x00, 0x00, 0x00, 0xcc, 0x48, 0x78, 0x70, 0x30},
    {0x00, 0x00, 0x00, 0xdb, 0x5b, 0x7e, 0x7e, 0x76},
    {0x00, 0x00, 0x00, 0xd8, 0x78, 0x30, 0x78, 0xc8},
    {0x00, 0x00, 0x00, 0xcc, 0x48, 0x78, 0x78, 0x30},
    {0x00, 0x00, 0x00, 0xf8, 0x38, 0x30, 0x60, 0xf8},
    {0x18, 0x30, 0x30, 0x70, 0x30, 0x30, 0x30, 0x18},
    {0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},
    {0x70, 0x30, 0x30, 0x18, 0x30, 0x30, 0x30, 0x70},
    {0x00, 0x00, 0x00, 0x00, 0x72, 0x1c, 0x00, 0x00},
    {0x00, 0x38, 0x38, 0x78, 0x6c, 0x6c, 0xfe, 0xc6},
    {0x00, 0x7c, 0x60, 0x60, 0x7c, 0x6c, 0x6c, 0x7c},
    {0x00, 0x78, 0x6c, 0x6c, 0x7c, 0x6c, 0x6c, 0x7c},
    {0x00, 0x7c, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60},
    {0x00, 0x3e, 0x36, 0x36, 0x36, 0x26, 0x66, 0xff},
    {0x00, 0x7c, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7c},
    {0x00, 0x64, 0x35, 0x3f, 0x3f, 0x3f, 0x64, 0xc4},
    {0x00, 0x78, 0x0c, 0x0c, 0x38, 0x0c, 0x0c, 0xf8},
    {0x00, 0x6e, 0x6e, 0x7e, 0x7e, 0x76, 0x76, 0x66},
    {0x00, 0x6e, 0x6e, 0x7e, 0x7e, 0x76, 0x76, 0x66},
    {0x00, 0x66, 0x6c, 0x78, 0x78, 0x7c, 0x6c, 0x66},
    {0x00, 0x3e, 0x36, 0x36, 0x36, 0x36, 0x66, 0xc6},
    {0x00, 0x63, 0x77, 0x77, 0x7f, 0x7f, 0x6b, 0x63},
    {0x00, 0x66, 0x66, 0x66, 0x7e, 0x66, 0x66, 0x66},
    {0x00, 0x3c, 0x66, 0xc6, 0xc6, 0xc6, 0x66, 0x3c},
    {0x00, 0x7e, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66},
    {0x00, 0x7c, 0x6c, 0x6c, 0x7c, 0x60, 0x60, 0x60},
    {0x00, 0x3c, 0x64, 0xc0, 0xc0, 0xc0, 0x64, 0x3c},
    {0x00, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30},
    {0x00, 0xc6, 0x6c, 0x7c, 0x38, 0x38, 0x30, 0x70},
    {0x00, 0x18, 0x7e, 0x7b, 0xd9, 0x7b, 0x7e, 0x18},
    {0x00, 0xcc, 0x6c, 0x38, 0x38, 0x38, 0x6c, 0xc6},
    {0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7f},
    {0x00, 0x66, 0x66, 0x66, 0x7e, 0x06, 0x06, 0x06},
    {0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7f},
    {0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7f},
    {0x00, 0x70, 0xf0, 0x30, 0x3e, 0x3b, 0x3b, 0x3e},
    {0x00, 0x61, 0x61, 0x61, 0x7d, 0x6d, 0x6d, 0x7d},
    {0x00, 0x60, 0x60, 0x60, 0x7c, 0x6c, 0x6c, 0x7c},
    {0x00, 0x70, 0xf8, 0x0c, 0x7c, 0x0c, 0x1c, 0x78},
    {0x00, 0x67, 0x6c, 0x78, 0x78, 0x78, 0x6c, 0x67},
    {0x00, 0x7c, 0x6c, 0x6c, 0x7c, 0x3c, 0x6c, 0x6c},
    {0x00, 0x00, 0x00, 0x78, 0x08, 0x78, 0xd8, 0x78},
    {0x38, 0x40, 0xc0, 0xf8, 0xec, 0xcc, 0x6c, 0x78},
    {0x00, 0x00, 0x00, 0x78, 0x58, 0x78, 0x48, 0x78},
    {0x00, 0x00, 0x00, 0x78, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x00, 0x00, 0x3c, 0x2c, 0x6c, 0x6c, 0xfe},
    {0x00, 0x00, 0x00, 0x78, 0xcc, 0xfc, 0xc0, 0x7c},
    {0x00, 0x00, 0x00, 0x6b, 0x3e, 0x3e, 0x7f, 0xc9},
    {0x00, 0x00, 0x00, 0x70, 0x18, 0x70, 0x18, 0xf0},
    {0x00, 0x00, 0x00, 0x5c, 0x7c, 0x7c, 0x6c, 0x6c},
    {0x00, 0x30, 0x00, 0x5c, 0x7c, 0x7c, 0x6c, 0x6c},
    {0x00, 0x00, 0x00, 0x58, 0x70, 0x70, 0x78, 0x4c},
    {0x00, 0x00, 0x00, 0x7c, 0x6c, 0x6c, 0x6c, 0xcc},
    {0x00, 0x00, 0x00, 0x6e, 0x6e, 0x7e, 0x7e, 0x46},
    {0x00, 0x00, 0x00, 0x4c, 0x4c, 0x7c, 0x4c, 0x4c},
    {0x00, 0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0x78},
    {0x00, 0x00, 0x00, 0x7c, 0x4c, 0x4c, 0x4c, 0x4c},
    {0x00, 0x00, 0x00, 0x78, 0x6c, 0x4c, 0x6c, 0x78},
    {0x00, 0x00, 0x00, 0x78, 0x60, 0xc0, 0x60, 0x78},
    {0x00, 0x00, 0x00, 0xf8, 0x30, 0x30, 0x30, 0x30},
    {0x00, 0x00, 0x00, 0xcc, 0x48, 0x78, 0x78, 0x30},
    {0x08, 0x08, 0x08, 0x7f, 0x7b, 0xc9, 0x7b, 0x7f},
    {0x00, 0x00, 0x00, 0xd8, 0x78, 0x30, 0x78, 0xc8},
    {0x00, 0x00, 0x00, 0x4c, 0x4c, 0x4c, 0x4c, 0x7c},
    {0x00, 0x00, 0x00, 0xd8, 0xd8, 0x78, 0x18, 0x18},
    {0x00, 0x00, 0x00, 0x4d, 0x4d, 0x4d, 0x4d, 0x7f},
    {0x00, 0x00, 0x00, 0x4d, 0x4d, 0x4d, 0x4d, 0x7f},
    {0x00, 0x00, 0x00, 0xf0, 0x30, 0x3c, 0x34, 0x3c},
    {0x00, 0x00, 0x00, 0x43, 0x43, 0x7b, 0x4f, 0x7b},
    {0x00, 0x00, 0x00, 0x40, 0x40, 0x78, 0x48, 0x78},
    {0x00, 0x00, 0x00, 0x70, 0x18, 0x78, 0x18, 0x70},
    {0x00, 0x00, 0x00, 0x4e, 0x5b, 0x79, 0x5b, 0x4e},
    {0x00, 0x00, 0x00, 0x78, 0x58, 0x78, 0x78, 0xd8},
};

static int bar_glyph_index(uint32_t codepoint) {
    if (codepoint >= 32 && codepoint < 127) {
        return static_cast<int>(codepoint) - 32;
    }
    if (codepoint >= 0x410 && codepoint <= 0x44F) {
        return 95 + static_cast<int>(codepoint - 0x410);
    }
    if (codepoint == 0x401) {
        return 95 + 6;
    }
    if (codepoint == 0x451) {
        return 95 + 37;
    }
    return -1;
}

static std::vector<uint32_t> decode_utf8(const std::string& text) {
    std::vector<uint32_t> codepoints;
    for (size_t i = 0; i < text.size();) {
        const uint8_t byte = static_cast<uint8_t>(text[i]);
        uint32_t codepoint = byte;
        size_t length = 1;
        if ((byte & 0xE0) == 0xC0) {
            codepoint = byte & 0x1F;
            length = 2;
        } else if ((byte & 0xF0) == 0xE0) {
            codepoint = byte & 0x0F;
            length = 3;
        } else if ((byte & 0xF8) == 0xF0) {
            codepoint = byte & 0x07;
            length = 4;
        }
        if (i + length > text.size()) {
            break;
        }
        for (size_t k = 1; k < length; k++) {
            codepoint =
                (codepoint << 6) | (static_cast<uint8_t>(text[i + k]) & 0x3F);
        }
        codepoints.push_back(codepoint);
        i += length;
    }
    return codepoints;
}

static GLuint compile_program(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexSource, nullptr);
    glCompileShader(vertexShader);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);
    glCompileShader(fragmentShader);
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return program;
}

static void push_quad(
    std::vector<float>& target, float x, float y, float w, float h, float u0,
    float v0, float u1, float v1, const glm::vec4& color
) {
    const float x1 = x + w;
    const float y1 = y + h;
    const float vertices[] {
        x,  y,  u0, v0, color.r, color.g, color.b, color.a,
        x1, y,  u1, v0, color.r, color.g, color.b, color.a,
        x,  y1, u0, v1, color.r, color.g, color.b, color.a,
        x1, y,  u1, v0, color.r, color.g, color.b, color.a,
        x1, y1, u1, v1, color.r, color.g, color.b, color.a,
        x,  y1, u0, v1, color.r, color.g, color.b, color.a
    };
    target.insert(target.end(), vertices, vertices + 48);
}

static bool serverDecorations = false;

static void zxdg_decoration_configure(
    void*, zxdg_toplevel_decoration_v1*, uint32_t mode
);

static void on_display_error();

static int keycode_from_keysym(xkb_keysym_t sym) {
    switch (sym) {
        case XKB_KEY_Escape: return GLFW_KEY_ESCAPE;
        case XKB_KEY_Return: return GLFW_KEY_ENTER;
        case XKB_KEY_Tab: return GLFW_KEY_TAB;
        case XKB_KEY_BackSpace: return GLFW_KEY_BACKSPACE;
        case XKB_KEY_Insert: return GLFW_KEY_INSERT;
        case XKB_KEY_Delete: return GLFW_KEY_DELETE;
        case XKB_KEY_Left: return GLFW_KEY_LEFT;
        case XKB_KEY_Right: return GLFW_KEY_RIGHT;
        case XKB_KEY_Up: return GLFW_KEY_UP;
        case XKB_KEY_Down: return GLFW_KEY_DOWN;
        case XKB_KEY_Page_Up: return GLFW_KEY_PAGE_UP;
        case XKB_KEY_Page_Down: return GLFW_KEY_PAGE_DOWN;
        case XKB_KEY_Home: return GLFW_KEY_HOME;
        case XKB_KEY_End: return GLFW_KEY_END;
        case XKB_KEY_Caps_Lock: return GLFW_KEY_CAPS_LOCK;
        case XKB_KEY_Scroll_Lock: return GLFW_KEY_SCROLL_LOCK;
        case XKB_KEY_Num_Lock: return GLFW_KEY_NUM_LOCK;
        case XKB_KEY_Print: return GLFW_KEY_PRINT_SCREEN;
        case XKB_KEY_Pause: return GLFW_KEY_PAUSE;
        case XKB_KEY_Menu: return GLFW_KEY_MENU;
        case XKB_KEY_Shift_L: return GLFW_KEY_LEFT_SHIFT;
        case XKB_KEY_Control_L: return GLFW_KEY_LEFT_CONTROL;
        case XKB_KEY_Alt_L: return GLFW_KEY_LEFT_ALT;
        case XKB_KEY_Super_L: return GLFW_KEY_LEFT_SUPER;
        case XKB_KEY_Shift_R: return GLFW_KEY_RIGHT_SHIFT;
        case XKB_KEY_Control_R: return GLFW_KEY_RIGHT_CONTROL;
        case XKB_KEY_Alt_R: return GLFW_KEY_RIGHT_ALT;
        case XKB_KEY_Super_R: return GLFW_KEY_RIGHT_SUPER;
        case XKB_KEY_space: return GLFW_KEY_SPACE;
        case XKB_KEY_apostrophe: return GLFW_KEY_APOSTROPHE;
        case XKB_KEY_comma: return GLFW_KEY_COMMA;
        case XKB_KEY_minus: return GLFW_KEY_MINUS;
        case XKB_KEY_period: return GLFW_KEY_PERIOD;
        case XKB_KEY_slash: return GLFW_KEY_SLASH;
        case XKB_KEY_semicolon: return GLFW_KEY_SEMICOLON;
        case XKB_KEY_equal: return GLFW_KEY_EQUAL;
        case XKB_KEY_bracketleft: return GLFW_KEY_LEFT_BRACKET;
        case XKB_KEY_backslash: return GLFW_KEY_BACKSLASH;
        case XKB_KEY_bracketright: return GLFW_KEY_RIGHT_BRACKET;
        case XKB_KEY_grave: return GLFW_KEY_GRAVE_ACCENT;
        case XKB_KEY_KP_0: return GLFW_KEY_KP_0;
        case XKB_KEY_KP_1: return GLFW_KEY_KP_1;
        case XKB_KEY_KP_2: return GLFW_KEY_KP_2;
        case XKB_KEY_KP_3: return GLFW_KEY_KP_3;
        case XKB_KEY_KP_4: return GLFW_KEY_KP_4;
        case XKB_KEY_KP_5: return GLFW_KEY_KP_5;
        case XKB_KEY_KP_6: return GLFW_KEY_KP_6;
        case XKB_KEY_KP_7: return GLFW_KEY_KP_7;
        case XKB_KEY_KP_8: return GLFW_KEY_KP_8;
        case XKB_KEY_KP_9: return GLFW_KEY_KP_9;
        case XKB_KEY_KP_Decimal: return GLFW_KEY_KP_DECIMAL;
        case XKB_KEY_KP_Divide: return GLFW_KEY_KP_DIVIDE;
        case XKB_KEY_KP_Multiply: return GLFW_KEY_KP_MULTIPLY;
        case XKB_KEY_KP_Subtract: return GLFW_KEY_KP_SUBTRACT;
        case XKB_KEY_KP_Add: return GLFW_KEY_KP_ADD;
        case XKB_KEY_KP_Enter: return GLFW_KEY_KP_ENTER;
        case XKB_KEY_KP_Equal: return GLFW_KEY_KP_EQUAL;
    }
    if (sym >= XKB_KEY_a && sym <= XKB_KEY_z) {
        return GLFW_KEY_A + (sym - XKB_KEY_a);
    }
    if (sym >= XKB_KEY_A && sym <= XKB_KEY_Z) {
        return GLFW_KEY_A + (sym - XKB_KEY_A);
    }
    if (sym >= XKB_KEY_0 && sym <= XKB_KEY_9) {
        return GLFW_KEY_0 + (sym - XKB_KEY_0);
    }
    if (sym >= XKB_KEY_F1 && sym <= XKB_KEY_F12) {
        return GLFW_KEY_F1 + (sym - XKB_KEY_F1);
    }
    return 0;
}

static void dispatch_events(int timeoutMs) {
    wl_display_flush(state.display);
    while (wl_display_prepare_read(state.display) != 0) {
        wl_display_dispatch_pending(state.display);
        wl_display_flush(state.display);
    }
    pollfd descriptor {wl_display_get_fd(state.display), POLLIN, 0};
    if (poll(&descriptor, 1, timeoutMs) > 0) {
        if (wl_display_read_events(state.display) == -1) {
            wl_display_cancel_read(state.display);
        }
    } else {
        wl_display_cancel_read(state.display);
    }
    wl_display_dispatch_pending(state.display);
    if (wl_display_get_error(state.display) != 0) {
        on_display_error();
    }
}

class WaylandInput : public BaseInput {
public:
    void pollEvents(bool waitForRefresh) override;

    const char* getClipboardText() const override {
        if (state.dataDevice == nullptr || selection == nullptr) {
            return clipboard.c_str();
        }
        const char* mime = pickMime();
        if (mime == nullptr) {
            return clipboard.c_str();
        }
        int fds[2];
        if (pipe(fds) == -1) {
            return clipboard.c_str();
        }
        wl_data_offer_receive(selection, mime, fds[1]);
        close(fds[1]);
        wl_display_flush(state.display);

        clipboard.clear();
        char buffer[4096];
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(500);
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd descriptor {fds[0], POLLIN, 0};
            if (poll(&descriptor, 1, 50) <= 0) {
                wl_display_dispatch_pending(state.display);
                continue;
            }
            ssize_t length = read(fds[0], buffer, sizeof(buffer));
            if (length <= 0) {
                break;
            }
            clipboard.append(buffer, length);
        }
        close(fds[0]);
        return clipboard.c_str();
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
        wl_data_device_set_selection(state.dataDevice, source, state.inputSerial);
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

    void onRelativeMotion(double dx, double dy) {
        delta.x += dx;
        delta.y += dy;
    }

    void onKey(uint32_t key, bool pressed) {
        if (state.xkbState == nullptr || state.xkbKeymap == nullptr) {
            return;
        }
        xkb_keycode_t code = key + 8;
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
    mutable std::string clipboard;
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
            case CursorShape::NOT_ALLOWED:
                return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_NOT_ALLOWED;
        }
        return WP_CURSOR_SHAPE_DEVICE_V1_SHAPE_DEFAULT;
    }

    static void dispatch(bool block) {
        dispatch_events(block ? 500 : 0);
    }

    static double now() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()
        ).count();
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
            if (codepoint != 0) {
                codepoints.push_back(codepoint);
            }
            codepoint = (bytes[i] & 0x80) ? (bytes[i] & 0x1F) : bytes[i];
        }
        if (codepoint != 0) {
            codepoints.push_back(codepoint);
        }
    }

    void updateRepeat() {
        if (repeatKeycode == 0 || !state.xkbState) {
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
                    if (!text.empty() && write(fd, text.data(), text.size()) < 0) {
                        logger.warning() << "could not write to the clipboard";
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

class WaylandWindow : public Window {
public:
    WaylandInput& input;
    DisplaySettings* settings;

    WaylandWindow(
        WaylandInput& input, DisplaySettings* settings, int width, int height
    )
        : Window({width, height}), input(input), settings(settings) {
        scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
    }

    ~WaylandWindow() override {
        if (state.idleInhibitor) {
            zwp_idle_inhibitor_v1_destroy(state.idleInhibitor);
        }
        if (state.lockedPointer) {
            zwp_locked_pointer_v1_destroy(state.lockedPointer);
        }
        if (state.relativePointer) {
            zwp_relative_pointer_v1_destroy(state.relativePointer);
        }
        if (state.cursorShapeDevice) {
            wp_cursor_shape_device_v1_destroy(state.cursorShapeDevice);
        }
        if (frameCallback) {
            wl_callback_destroy(frameCallback);
            frameCallback = nullptr;
        }
        if (cacheTexture) {
            glDeleteTextures(1, &cacheTexture);
        }
        if (cacheFbo) {
            glDeleteFramebuffers(1, &cacheFbo);
        }
        if (stretchProgram) {
            glDeleteProgram(stretchProgram);
        }
        if (stretchVbo) {
            glDeleteBuffers(1, &stretchVbo);
        }
        if (stretchVao) {
            glDeleteVertexArrays(1, &stretchVao);
        }
        if (barFontTexture) {
            glDeleteTextures(1, &barFontTexture);
        }
        if (barIconTexture) {
            glDeleteTextures(1, &barIconTexture);
        }
        if (barProgram) {
            glDeleteProgram(barProgram);
        }
        if (barVbo) {
            glDeleteBuffers(1, &barVbo);
        }
        if (barVao) {
            glDeleteVertexArrays(1, &barVao);
        }
        if (state.dataDevice) {
            wl_data_device_destroy(state.dataDevice);
        }
        if (state.decoration) {
            zxdg_toplevel_decoration_v1_destroy(state.decoration);
        }
        if (state.toplevel) {
            xdg_toplevel_destroy(state.toplevel);
        }
        if (state.xdgSurface) {
            xdg_surface_destroy(state.xdgSurface);
        }
        if (eglSurface != EGL_NO_SURFACE) {
            eglDestroySurface(eglDisplay, eglSurface);
        }
        if (eglWindow) {
            wl_egl_window_destroy(eglWindow);
        }
        if (state.surface) {
            wl_surface_destroy(state.surface);
        }
        if (eglContext != EGL_NO_CONTEXT) {
            eglDestroyContext(eglDisplay, eglContext);
        }
        if (eglDisplay != EGL_NO_DISPLAY) {
            eglTerminate(eglDisplay);
        }
    }

    double time() override {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    void swapBuffers() override {
        updateResizing();
        if (resizing && !presentPending && !frameDone) {
            resetScissor();
            return;
        }
        if (resizing) {
            if (renderPending) {
                cacheContent();
                renderPending = false;
            } else {
                drawStretched();
            }
        } else {
            cacheContent();
        }
        drawBar();
        requestFrame();
        eglSwapBuffers(eglDisplay, eglSurface);
        resetScissor();
        frameDone = false;
        presentPending = false;
        const double current = now();
        if (lastPresent > 0.0) {
            const double interval = current - lastPresent;
            if (interval > 0.001 && interval < 1.0 && !resizing &&
                frameOnCallback) {
                frameInterval = std::clamp(
                    frameInterval > 0.0 ? frameInterval * 0.8 + interval * 0.2
                                        : interval,
                    1.0 / 240.0,
                    1.0 / 30.0
                );
            }
            if (resizeLogging) {
                summarySum += interval;
                summaryMax = std::max(summaryMax, interval);
            }
        }
        lastPresent = current;
        nextFrameTime = current + frameInterval;
        if (resizeLogging) {
            summaryFrames++;
        }
        if (resizing) {
            const double elapsed = now() - configureTime;
            resizeTimeSum += elapsed;
            resizeTimeMax = std::max(resizeTimeMax, elapsed);
            resizePresents++;
        }
        lastSwapEnd = now();
        if (framerate > 0 && !resizing) {
            auto elapsed = time() - prevSwap;
            auto frameTime = 1.0 / framerate;
            if (elapsed < frameTime) {
                platform::sleep(
                    static_cast<size_t>((frameTime - elapsed) * 1000)
                );
            }
        }
        prevSwap = time();
    }

    void setShouldRefresh() override {
        shouldRefresh = true;
    }

    bool checkShouldRefresh() override {
        if (shouldRefresh) {
            shouldRefresh = false;
            return true;
        }
        return false;
    }

    bool isMaximized() const override {
        return maximized;
    }

    bool isFocused() const override {
        return activated;
    }

    bool isIconified() const override {
        return suspended;
    }

    bool isFrameRequired() const override {
        return !resizing || renderPending;
    }

    bool isShouldClose() const override {
        return shouldClose;
    }

    void setShouldClose(bool flag) override {
        shouldClose = flag;
    }

    void setCursor(CursorShape shape) override {
        input.setCursorShape(shape);
    }

    void setMode(WindowMode mode) override {
        Window::mode = mode;
        if (input.isCursorLocked()) {
            input.toggleCursor();
        }
        switch (mode) {
            case WindowMode::FULLSCREEN:
                setFullscreen(true);
                setIdleInhibit(true);
                break;
            case WindowMode::BORDERLESS:
                setFullscreen(false);
                setMaximized(true);
                setIdleInhibit(true);
                break;
            case WindowMode::WINDOWED:
                setFullscreen(false);
                setMaximized(false);
                setIdleInhibit(false);
                resize(settings->width.get(), settings->height.get());
                break;
        }
        wl_surface_commit(state.surface);
    }

    WindowMode getMode() const override {
        return mode;
    }

    void focus() override {
        if (!focusWarning) {
            focusWarning = true;
            logger.warning() << "raising windows is not allowed on Wayland";
        }
    }

    void setTitle(const std::string& title) override {
        this->title = title;
        xdg_toplevel_set_title(state.toplevel, title.c_str());
    }

    void setFullscreen(bool enabled) {
        fullscreen = enabled;
        if (enabled) {
            xdg_toplevel_set_fullscreen(state.toplevel, nullptr);
        } else {
            xdg_toplevel_unset_fullscreen(state.toplevel);
        }
    }

    void setMaximized(bool enabled) {
        if (enabled) {
            xdg_toplevel_set_maximized(state.toplevel);
        } else {
            xdg_toplevel_unset_maximized(state.toplevel);
        }
    }

    bool isIconSupported() const override {
        return true;
    }

    void setIcon(const ImageData* image) override {
        if (image == nullptr || image->getData() == nullptr) {
            return;
        }
        iconWidth = static_cast<int>(image->getWidth());
        iconHeight = static_cast<int>(image->getHeight());
        if (iconWidth <= 0 || iconHeight <= 0) {
            return;
        }
        const size_t pixels = static_cast<size_t>(iconWidth) * iconHeight;
        const bool alpha = image->getFormat() == ImageFormat::RGBA8888;
        iconPixels.resize(pixels * 4);
        for (size_t i = 0; i < pixels; i++) {
            iconPixels[i * 4] = image->getData()[i * (alpha ? 4 : 3)];
            iconPixels[i * 4 + 1] = image->getData()[i * (alpha ? 4 : 3) + 1];
            iconPixels[i * 4 + 2] = image->getData()[i * (alpha ? 4 : 3) + 2];
            iconPixels[i * 4 + 3] = alpha ? image->getData()[i * 4 + 3] : 255;
        }
        if (barIconTexture != 0) {
            glDeleteTextures(1, &barIconTexture);
            barIconTexture = 0;
        }
    }

    void resize(int width, int height) {
        if (width <= 0 || height <= 0) {
            return;
        }
        if (size.x != width || size.y != height) {
            markResizing();
        }
        configureTime = now();
        presentPending = true;
        if (eglWindow) {
            wl_egl_window_resize(eglWindow, width, height, 0, 0);
        }
        glViewport(0, 0, width, height);
        size = {width, height};
        scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
    }

    void onConfigure(int32_t width, int32_t height, wl_array* states) {
        pendingWidth = width;
        pendingHeight = height;
        stateMaximized = false;
        stateSuspended = false;
        stateActivated = false;
        const auto* values = static_cast<const uint32_t*>(states->data);
        for (size_t i = 0; i < states->size / sizeof(uint32_t); i++) {
            switch (values[i]) {
                case XDG_TOPLEVEL_STATE_MAXIMIZED:
                    stateMaximized = true;
                    break;
                case XDG_TOPLEVEL_STATE_SUSPENDED:
                    stateSuspended = true;
                    break;
                case XDG_TOPLEVEL_STATE_ACTIVATED:
                    stateActivated = true;
                    break;
            }
        }
    }

    void applyConfigure() {
        if (pendingWidth > 0 && pendingHeight > 0) {
            resize(pendingWidth, pendingHeight);
        }
        maximized = stateMaximized;
        suspended = stateSuspended;
        activated = stateActivated;
        configured = true;
        setShouldRefresh();
    }

    void pushScissor(glm::vec4 area) override {
        if (scissorStack.empty()) {
            glEnable(GL_SCISSOR_TEST);
        }
        scissorStack.push(scissorArea);

        area.z += glm::ceil(area.x);
        area.w += glm::ceil(area.y);

        area.x = glm::max(area.x, scissorArea.x);
        area.y = glm::max(area.y, scissorArea.y);

        area.z = glm::min(area.z, scissorArea.z);
        area.w = glm::min(area.w, scissorArea.w);

        if (area.z < 0.0f || area.w < 0.0f) {
            glScissor(0, 0, 0, 0);
        } else {
            glScissor(
                area.x,
                size.y - area.w,
                std::max(0, static_cast<int>(glm::ceil(area.z - area.x))),
                std::max(0, static_cast<int>(glm::ceil(area.w - area.y)))
            );
        }
        scissorArea = area;
    }

    void popScissor() override {
        if (scissorStack.empty()) {
            logger.warning() << "extra Window::popScissor call";
            return;
        }
        glm::vec4 area = scissorStack.top();
        scissorStack.pop();
        if (area.z < 0.0f || area.w < 0.0f) {
            glScissor(0, 0, 0, 0);
        } else {
            glScissor(
                area.x,
                size.y - area.w,
                std::max(0, static_cast<int>(area.z - area.x)),
                std::max(0, static_cast<int>(area.w - area.y))
            );
        }
        if (scissorStack.empty()) {
            glDisable(GL_SCISSOR_TEST);
        }
        scissorArea = area;
    }

    void resetScissor() override {
        scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
        scissorStack = std::stack<glm::vec4>();
        glDisable(GL_SCISSOR_TEST);
    }

    std::unique_ptr<ImageData> takeScreenshot() override {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, size.x, size.y, GL_RGB, GL_UNSIGNED_BYTE, data.get());
        return std::make_unique<ImageData>(
            ImageFormat::RGB888, size.x, size.y, data.release()
        );
    }

    void setFramerate(int framerate) override {
        this->framerate = framerate;
    }

    void initStretchRenderer() {
        static const char* vertexSource =
            "#version 330 core\n"
            "layout(location = 0) in vec2 aPos;\n"
            "out vec2 vUV;\n"
            "void main() {\n"
            "    vUV = aPos * 0.5 + 0.5;\n"
            "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
            "}\n";
        static const char* fragmentSource =
            "#version 330 core\n"
            "in vec2 vUV;\n"
            "out vec4 color;\n"
            "uniform sampler2D uTexture;\n"
            "void main() {\n"
            "    color = texture(uTexture, vUV);\n"
            "}\n";
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexSource, nullptr);
        glCompileShader(vertexShader);
        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);
        glCompileShader(fragmentShader);
        stretchProgram = glCreateProgram();
        glAttachShader(stretchProgram, vertexShader);
        glAttachShader(stretchProgram, fragmentShader);
        glLinkProgram(stretchProgram);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        const float vertices[] {-1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f};
        glGenVertexArrays(1, &stretchVao);
        glGenBuffers(1, &stretchVbo);
        glBindVertexArray(stretchVao);
        glBindBuffer(GL_ARRAY_BUFFER, stretchVbo);
        glBufferData(
            GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW
        );
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glBindVertexArray(0);
        glGenFramebuffers(1, &cacheFbo);
        glGenTextures(1, &cacheTexture);
    }

    void resizeCache(int width, int height) {
        if (cacheWidth == width && cacheHeight == height) {
            return;
        }
        cacheWidth = width;
        cacheHeight = height;
        glBindTexture(GL_TEXTURE_2D, cacheTexture);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
            GL_UNSIGNED_BYTE, nullptr
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindFramebuffer(GL_FRAMEBUFFER, cacheFbo);
        glFramebufferTexture2D(
            GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, cacheTexture,
            0
        );
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void cacheContent() {
        if (stretchProgram == 0) {
            return;
        }
        resizeCache(size.x, size.y);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cacheFbo);
        glBlitFramebuffer(
            0, 0, size.x, size.y, 0, 0, size.x, size.y, GL_COLOR_BUFFER_BIT,
            GL_NEAREST
        );
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        lastCacheTime = now();
    }

    void drawStretched() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, size.x, size.y);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_CULL_FACE);
        glDisable(GL_SCISSOR_TEST);
        if (stretchProgram == 0 || cacheWidth == 0) {
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            return;
        }
        glUseProgram(stretchProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, cacheTexture);
        glUniform1i(glGetUniformLocation(stretchProgram, "uTexture"), 0);
        glBindVertexArray(stretchVao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
    }

    // libdecor redraws the whole decoration inside libdecor_frame_commit,
    // which is also the only place where it acks the configure
    void enableOwnDecorations() {
        if (!decorations_enabled()) {
            return;
        }
        barEnabled = true;
        setShouldRefresh();
    }

    bool barVisible() const {
        return barEnabled && !fullscreen && Window::mode != WindowMode::BORDERLESS;
    }

    void initBarRenderer() {
        if (barProgram != 0) {
            return;
        }
        static const char* vertexSource =
            "#version 330 core\n"
            "layout(location = 0) in vec2 aPos;\n"
            "layout(location = 1) in vec2 aUv;\n"
            "layout(location = 2) in vec4 aColor;\n"
            "uniform vec2 uViewport;\n"
            "out vec2 vUv;\n"
            "out vec4 vColor;\n"
            "void main() {\n"
            "    vec2 ndc = vec2(aPos.x / uViewport.x * 2.0 - 1.0,"
            " 1.0 - aPos.y / uViewport.y * 2.0);\n"
            "    gl_Position = vec4(ndc, 0.0, 1.0);\n"
            "    vUv = aUv;\n"
            "    vColor = aColor;\n"
            "}\n";
        static const char* fragmentSource =
            "#version 330 core\n"
            "in vec2 vUv;\n"
            "in vec4 vColor;\n"
            "out vec4 outColor;\n"
            "uniform sampler2D uTexture;\n"
            "uniform int uMode;\n"
            "void main() {\n"
            "    if (uMode == 1) {\n"
            "        outColor = vec4(vColor.rgb,"
            " vColor.a * texture(uTexture, vUv).r);\n"
            "    } else if (uMode == 2) {\n"
            "        outColor = texture(uTexture, vUv) * vColor;\n"
            "    } else {\n"
            "        outColor = vColor;\n"
            "    }\n"
            "}\n";
        barProgram = compile_program(vertexSource, fragmentSource);
        barViewportLoc = glGetUniformLocation(barProgram, "uViewport");
        barTextureLoc = glGetUniformLocation(barProgram, "uTexture");
        barModeLoc = glGetUniformLocation(barProgram, "uMode");
        glGenVertexArrays(1, &barVao);
        glGenBuffers(1, &barVbo);
        glBindVertexArray(barVao);
        glBindBuffer(GL_ARRAY_BUFFER, barVbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            0, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), nullptr
        );
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
            reinterpret_cast<void*>(2 * sizeof(float))
        );
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(
            2, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
            reinterpret_cast<void*>(4 * sizeof(float))
        );
        glBindVertexArray(0);
        glGenTextures(1, &barFontTexture);
        glBindTexture(GL_TEXTURE_2D, barFontTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        std::vector<uint8_t> pixels(BAR_GLYPHS * BAR_FONT_W * BAR_FONT_H, 0);
        for (int glyph = 0; glyph < BAR_GLYPHS; glyph++) {
            for (int y = 0; y < BAR_FONT_H; y++) {
                for (int x = 0; x < BAR_FONT_W; x++) {
                    if (BAR_FONT[glyph][y] & (1 << (7 - x))) {
                        pixels[y * BAR_GLYPHS * BAR_FONT_W + glyph * BAR_FONT_W + x] =
                            255;
                    }
                }
            }
        }
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_R8, BAR_GLYPHS * BAR_FONT_W, BAR_FONT_H, 0,
            GL_RED, GL_UNSIGNED_BYTE, pixels.data()
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void initBarIconTexture() {
        if (iconPixels.empty() || barIconTexture != 0) {
            return;
        }
        glGenTextures(1, &barIconTexture);
        glBindTexture(GL_TEXTURE_2D, barIconTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA8, iconWidth, iconHeight, 0, GL_RGBA,
            GL_UNSIGNED_BYTE, iconPixels.data()
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void barDrawText(
        float x, float y, const std::vector<uint32_t>& codepoints, int maxChars,
        const glm::vec4& color
    ) {
        const float step = 1.0f / static_cast<float>(BAR_GLYPHS * BAR_FONT_W);
        float cursor = x;
        for (int i = 0; i < static_cast<int>(codepoints.size()) && i < maxChars; i++) {
            const int index = bar_glyph_index(codepoints[i]);
            if (index >= 0) {
                push_quad(
                    barText, cursor, y, BAR_FONT_W, BAR_FONT_H,
                    index * BAR_FONT_W * step, 0.0f,
                    (index * BAR_FONT_W + BAR_FONT_W) * step, 1.0f, color
                );
            }
            cursor += BAR_FONT_W;
        }
    }

    void barDrawBatch(const std::vector<float>& data, int mode, GLuint texture) {
        if (data.empty()) {
            return;
        }
        glUniform1i(barModeLoc, mode);
        glBindTexture(GL_TEXTURE_2D, texture);
        glBufferData(
            GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(),
            GL_STREAM_DRAW
        );
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(data.size() / 8));
    }

    void barFlush() {
        glUseProgram(barProgram);
        glUniform2f(
            barViewportLoc, static_cast<float>(size.x), static_cast<float>(size.y)
        );
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_SCISSOR_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(barVao);
        glBindBuffer(GL_ARRAY_BUFFER, barVbo);
        glActiveTexture(GL_TEXTURE0);
        glUniform1i(barTextureLoc, 0);
        barDrawBatch(barRects, 0, barFontTexture);
        barDrawBatch(barIcons, 2, barIconTexture);
        barDrawBatch(barText, 1, barFontTexture);
        glBindVertexArray(0);
        glDisable(GL_BLEND);
    }

    // the bar is drawn over the content surface, so the window size
    // reported to the engine stays the same
    void drawBar() {
        if (!barVisible()) {
            return;
        }
        initBarRenderer();
        initBarIconTexture();
        barRects.clear();
        barText.clear();
        barIcons.clear();

        const float width = static_cast<float>(size.x);
        const float height = static_cast<float>(BAR_HEIGHT);
        const glm::vec4 background(0.16f, 0.16f, 0.17f, 1.0f);
        const glm::vec4 border(0.09f, 0.09f, 0.10f, 1.0f);
        const glm::vec4 ink(0.87f, 0.87f, 0.88f, 1.0f);
        const glm::vec4 hover(0.27f, 0.27f, 0.29f, 1.0f);
        const glm::vec4 closeHover(0.72f, 0.16f, 0.11f, 1.0f);

        push_quad(barRects, 0, 0, width, height, 0, 0, 0, 0, background);
        push_quad(barRects, 0, height - 1, width, 1, 0, 0, 0, 0, border);

        float textX = 10.0f;
        if (barIconTexture != 0 && iconWidth > 0) {
            push_quad(
                barIcons, 7, (height - BAR_ICON_SIZE) * 0.5f, BAR_ICON_SIZE,
                BAR_ICON_SIZE, 0, 1, 1, 0, glm::vec4(1.0f)
            );
            textX = 7 + BAR_ICON_SIZE + 8;
        }
        const float buttonsX = width - BAR_BUTTON * 3;
        const int maxChars =
            static_cast<int>((buttonsX - 8 - textX) / BAR_FONT_W);
        if (maxChars > 0) {
            barDrawText(
                textX, (height - BAR_FONT_H) * 0.5f, decode_utf8(title),
                maxChars, ink
            );
        }

        for (int i = 0; i < 3; i++) {
            const float bx = width - BAR_BUTTON * (i + 1);
            if (hoveredButton == i) {
                push_quad(
                    barRects, bx, 0, BAR_BUTTON, height - 1, 0, 0, 0, 0,
                    i == 0 ? closeHover : hover
                );
            }
            const float cx = bx + BAR_BUTTON * 0.5f;
            const float cy = height * 0.5f;
            if (i == 0) {
                for (int s = 0; s < 7; s++) {
                    push_quad(barRects, cx - 4 + s, cy - 4 + s, 1.6f, 1.6f, 0, 0, 0, 0, ink);
                    push_quad(barRects, cx + 3 - s, cy - 4 + s, 1.6f, 1.6f, 0, 0, 0, 0, ink);
                }
            } else if (i == 1 && !maximized) {
                push_quad(barRects, cx - 4.5f, cy - 4.5f, 9, 1.6f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx - 4.5f, cy + 2.9f, 9, 1.6f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx - 4.5f, cy - 4.5f, 1.6f, 9, 0, 0, 0, 0, ink);
                push_quad(barRects, cx + 2.9f, cy - 4.5f, 1.6f, 9, 0, 0, 0, 0, ink);
            } else if (i == 1) {
                push_quad(barRects, cx - 3.5f, cy - 5.5f, 8, 1.4f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx + 3.1f, cy - 5.5f, 1.4f, 5.5f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx - 5.5f, cy - 1.2f, 8, 1.4f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx - 5.5f, cy - 1.2f, 1.4f, 6.2f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx + 1.1f, cy - 1.2f, 1.4f, 6.2f, 0, 0, 0, 0, ink);
                push_quad(barRects, cx - 5.5f, cy + 3.6f, 8, 1.4f, 0, 0, 0, 0, ink);
            } else {
                push_quad(barRects, cx - 5, cy - 0.8f, 10, 1.6f, 0, 0, 0, 0, ink);
            }
        }
        barFlush();
    }

    int barButtonAt(double x, double y) const {
        if (!barVisible() || y < 0.0 || y >= BAR_HEIGHT) {
            return -1;
        }
        for (int i = 0; i < 3; i++) {
            const double bx = size.x - BAR_BUTTON * (i + 1);
            if (x >= bx && x < bx + BAR_BUTTON) {
                return i;
            }
        }
        return -1;
    }

    uint32_t barEdgeAt(double x, double y) const {
        uint32_t edges = 0;
        if (x < BAR_EDGE) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
        } else if (x >= size.x - BAR_EDGE) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
        }
        if (barVisible() && y < BAR_EDGE) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_TOP;
        } else if (y >= size.y - BAR_EDGE) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
        }
        return edges;
    }

    static CursorShape edgeCursor(uint32_t edges) {
        const bool left = edges & XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
        const bool right = edges & XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
        const bool top = edges & XDG_TOPLEVEL_RESIZE_EDGE_TOP;
        const bool bottom = edges & XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
        if ((left && top) || (right && bottom)) {
            return CursorShape::NWSE_RESIZE;
        }
        if ((right && top) || (left && bottom)) {
            return CursorShape::NESW_RESIZE;
        }
        if (left || right) {
            return CursorShape::EW_RESIZE;
        }
        if (top || bottom) {
            return CursorShape::NS_RESIZE;
        }
        return CursorShape::ARROW;
    }

    bool handleBarMotion(double x, double y) {
        pointerX = x;
        pointerY = y;
        const int button = barButtonAt(x, y);
        if (barVisible() && y >= 0.0 && y < BAR_HEIGHT) {
            hoveredButton = button;
            input.setOverrideCursor(
                button >= 0 ? CursorShape::POINTER : CursorShape::ARROW
            );
            return true;
        }
        hoveredButton = -1;
        const uint32_t edges = barEdgeAt(x, y);
        input.setOverrideCursor(
            edges != 0 ? std::optional<CursorShape>(edgeCursor(edges))
                       : std::nullopt
        );
        return false;
    }

    bool handleBarButton(uint32_t serial, bool pressed) {
        const double x = pointerX;
        const double y = pointerY;
        const int button = barButtonAt(x, y);
        if (button >= 0) {
            if (pressed) {
                if (button == 0) {
                    setShouldClose(true);
                } else if (button == 1) {
                    setMaximized(!maximized);
                } else {
                    xdg_toplevel_set_minimized(state.toplevel);
                }
            }
            return true;
        }
        if (barVisible() && y >= 0.0 && y < BAR_HEIGHT) {
            if (pressed) {
                const double current = now();
                if (current - lastBarPress < 0.4) {
                    lastBarPress = 0.0;
                    setMaximized(!maximized);
                } else {
                    lastBarPress = current;
                    xdg_toplevel_move(state.toplevel, state.seat, serial);
                }
            }
            return true;
        }
        const uint32_t edges = barEdgeAt(x, y);
        if (pressed && edges != 0) {
            xdg_toplevel_resize(state.toplevel, state.seat, serial, edges);
            return true;
        }
        return false;
    }

    void onPointerLeave() {
        hoveredButton = -1;
        input.setOverrideCursor(std::nullopt);
    }

    void onFrameDone() {
        frameCallback = nullptr;
        frameDone = true;
    }

    void requestFrame() {
        if (state.surface == nullptr || frameCallback != nullptr) {
            return;
        }
        frameCallback = wl_surface_frame(state.surface);
        wl_callback_add_listener(frameCallback, &frameListener(), this);
    }

    void paceFrame() {
        frameOnCallback = frameDone;
        if (lastSwapEnd > 0.0) {
            const double cost = now() - lastSwapEnd;
            if (cost > 0.0 && cost < 2.0) {
                frameCost = frameCost > 0.0 ? frameCost * 0.8 + cost * 0.2 : cost;
                cacheTimeout = std::clamp(
                    frameCost * 16.0, MIN_CACHE_TIMEOUT, MAX_CACHE_TIMEOUT
                );
            }
        }
        updateResizing();
        logSummary();
        if (resizing) {
            if (!renderPending && now() - lastResize > RESIZE_PAUSE &&
                now() - lastCacheTime > cacheTimeout) {
                renderPending = true;
            }
            dispatch_events(frameDone || presentPending ? 0 : 8);
            frameOnCallback = frameDone;
            return;
        }
        if (framerate != -1 || frameCallback == nullptr || frameDone) {
            return;
        }
        const double deadline = now() + FRAME_TIMEOUT;
        while (!frameDone && !resizing && now() < deadline) {
            dispatch_events(5);
        }
        while (!frameDone && !resizing && now() < nextFrameTime) {
            dispatch_events(5);
        }
        nextFrameTime = now() + frameInterval;
        frameOnCallback = frameDone;
    }

    void logSummary() {
        const double current = now();
        if (resizeLogging) {
            if (lastLoop > 0.0) {
                loopGapMax = std::max(loopGapMax, current - lastLoop);
            }
            lastLoop = current;
            loopCount++;
        }
        if (!resizeLogging || lastSummary <= 0.0) {
            if (resizeLogging) {
                lastSummary = current;
            }
            return;
        }
        if (current - lastSummary < 1.0) {
            return;
        }
        logger.info() << "loop " << loopCount << "/s (max gap "
                      << loopGapMax * 1000.0 << " ms), presents "
                      << summaryFrames << "/s (max gap "
                      << summaryMax * 1000.0 << " ms), resizing "
                      << (resizing ? "yes" : "no");
        lastSummary = current;
        loopCount = 0;
        loopGapMax = 0.0;
        summaryFrames = 0;
        summarySum = 0.0;
        summaryMax = 0.0;
    }

    void markResizing() {
        lastResize = now();
        if (!resizing) {
            resizing = true;
            resizePresents = 0;
            resizeTimeSum = 0.0;
            resizeTimeMax = 0.0;
            if (resizeLogging) {
                logger.info() << "resize started";
            }
            setShouldRefresh();
        }
    }

    void updateResizing() {
        if (resizing && now() - lastResize > RESIZE_TIMEOUT) {
            resizing = false;
            renderPending = true;
            if (resizeLogging) {
                double average = resizePresents > 0
                                     ? resizeTimeSum / resizePresents
                                     : 0.0;
                logger.info()
                    << "resize finished, presented " << resizePresents
                    << ", avg " << average * 1000.0 << " ms, max "
                    << resizeTimeMax * 1000.0 << " ms";
            }
        }
    }

    void setIdleInhibit(bool enabled) {
        if (enabled == (state.idleInhibitor != nullptr)) {
            return;
        }
        if (enabled && state.idleInhibitManager) {
            state.idleInhibitor = zwp_idle_inhibit_manager_v1_create_inhibitor(
                state.idleInhibitManager, state.surface
            );
        } else if (state.idleInhibitor) {
            zwp_idle_inhibitor_v1_destroy(state.idleInhibitor);
            state.idleInhibitor = nullptr;
        }
    }

    EGLDisplay eglDisplay = EGL_NO_DISPLAY;
    EGLContext eglContext = EGL_NO_CONTEXT;
    EGLSurface eglSurface = EGL_NO_SURFACE;
    wl_egl_window* eglWindow = nullptr;
    int32_t pendingWidth = 0;
    int32_t pendingHeight = 0;
    bool stateMaximized = false;
    bool stateSuspended = false;
    bool stateActivated = true;
    bool configured = false;
private:
    static constexpr double RESIZE_TIMEOUT = 0.4;
    static constexpr double FRAME_TIMEOUT = 0.025;
    static constexpr double RESIZE_PAUSE = 0.12;
    static constexpr double MIN_CACHE_TIMEOUT = 0.3;
    static constexpr double MAX_CACHE_TIMEOUT = 3.0;

    static const wl_callback_listener& frameListener() {
        static wl_callback_listener listener = [] {
            wl_callback_listener value {};
            value.done = [] (void* data, wl_callback* callback, uint32_t) {
                wl_callback_destroy(callback);
                static_cast<WaylandWindow*>(data)->onFrameDone();
            };
            return value;
        }();
        return listener;
    }

    static double now() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    int framerate = -1;
    double prevSwap = 0.0;
    double lastResize = 0.0;
    static inline bool resizeLogging = getenv("VOXEL_LOG_RESIZE") != nullptr;
    bool barEnabled = false;
    bool fullscreen = false;
    std::string title = "VoxelCore";
    std::vector<uint8_t> iconPixels;
    int iconWidth = 0;
    int iconHeight = 0;
    int hoveredButton = -1;
    double lastBarPress = 0.0;
    double pointerX = 0.0;
    double pointerY = 0.0;
    GLuint barProgram = 0;
    GLuint barVao = 0;
    GLuint barVbo = 0;
    GLuint barFontTexture = 0;
    GLuint barIconTexture = 0;
    GLint barViewportLoc = -1;
    GLint barTextureLoc = -1;
    GLint barModeLoc = -1;
    std::vector<float> barRects;
    std::vector<float> barText;
    std::vector<float> barIcons;
    bool frameOnCallback = false;
    double frameInterval = 0.0;
    double nextFrameTime = 0.0;
    double frameCost = 0.0;
    double cacheTimeout = 0.5;
    bool resizing = false;
    bool renderPending = true;
    bool presentPending = true;
    int resizePresents = 0;
    double configureTime = 0.0;
    double resizeTimeSum = 0.0;
    double resizeTimeMax = 0.0;
    double lastPresent = 0.0;
    double lastSummary = 0.0;
    double summarySum = 0.0;
    double summaryMax = 0.0;
    int summaryFrames = 0;
    double lastLoop = 0.0;
    double loopGapMax = 0.0;
    int loopCount = 0;
    wl_callback* frameCallback = nullptr;
    bool frameDone = false;
    GLuint stretchProgram = 0;
    GLuint stretchVao = 0;
    GLuint stretchVbo = 0;
    GLuint cacheFbo = 0;
    GLuint cacheTexture = 0;
    int cacheWidth = 0;
    int cacheHeight = 0;
    double lastCacheTime = 0.0;
    double lastSwapEnd = 0.0;
    bool shouldRefresh = true;
    bool shouldClose = false;
    bool maximized = false;
    bool suspended = false;
    bool activated = true;
    bool focusWarning = false;
    std::stack<glm::vec4> scissorStack;
    glm::vec4 scissorArea {};
};

void WaylandInput::pollEvents(bool waitForRefresh) {
    beginFrame();
    dispatch(waitForRefresh);
    if (window) {
        window->paceFrame();
    }
    updateRepeat();
    updateBindings();
}

static void on_display_error() {
    logger.error() << "the Wayland connection was lost";
    if (window) {
        window->setShouldClose(true);
    }
}

void WaylandInput::refreshWindow() {
    if (window) {
        window->setShouldRefresh();
    }
}

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
        input->setCursorPosition(x, y);
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
        input->setCursorPosition(x, y);
        input->refreshWindow();
    }
}

static void pointer_button(
    void*, wl_pointer*, uint32_t serial, uint32_t, uint32_t button,
    uint32_t buttonState
) {
    state.inputSerial = serial;
    int index = -1;
    switch (button) {
        case 0x110: index = 0; break;
        case 0x111: index = 1; break;
        case 0x112: index = 2; break;
        case 0x113: index = 3; break;
        case 0x114: index = 4; break;
        case 0x115: index = 5; break;
        case 0x116: index = 6; break;
        case 0x117: index = 7; break;
    }
    const bool pressed = buttonState == WL_POINTER_BUTTON_STATE_PRESSED;
    if (index == 0 && window && window->handleBarButton(serial, pressed)) {
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
        logger.error() << "could not parse the keyboard keymap";
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

static void keyboard_repeat_info(void*, wl_keyboard*, int32_t rate, int32_t delay) {
    state.repeatRate = rate > 0 ? rate : 25;
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
        listener.axis_value120 = ignore_event<>;
        listener.axis_relative_direction = ignore_event<>;
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

static void toplevel_configure(
    void*, xdg_toplevel*, int32_t width, int32_t height, wl_array* states
) {
    if (window) {
        window->onConfigure(width, height, states);
    }
}

static void toplevel_close(void*, xdg_toplevel*) {
    if (window) {
        window->setShouldClose(true);
    }
}

static void toplevel_configure_bounds(void*, xdg_toplevel*, int32_t, int32_t) {
}

static void toplevel_wm_capabilities(void*, xdg_toplevel*, wl_array*) {
}

static void xdg_surface_configure(void*, xdg_surface* surface, uint32_t serial) {
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

static void data_device_data_offer(void*, wl_data_device*, wl_data_offer* offer) {
    static const wl_data_offer_listener listener = [] {
        wl_data_offer_listener value {};
        value.offer = data_offer_offer;
        value.source_actions = ignore_event<>;
        value.action = ignore_event<>;
        return value;
    }();
    wl_data_offer_add_listener(offer, &listener, nullptr);
}

static void data_device_selection(void*, wl_data_device*, wl_data_offer* offer) {
    if (input == nullptr) {
        return;
    }
    if (input->selection) {
        wl_data_offer_destroy(input->selection);
    }
    input->selection = offer;
    input->selectionMimes = std::move(input->pendingMimes);
    input->pendingMimes.clear();
}

static void registry_global(
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
    } else if (strcmp(interface, wp_cursor_shape_manager_v1_interface.name) == 0) {
        state.cursorShapeManager = static_cast<wp_cursor_shape_manager_v1*>(
            wl_registry_bind(
                registry, name, &wp_cursor_shape_manager_v1_interface,
                std::min(version, 2u)
            )
        );
    } else if (strcmp(interface, zwp_pointer_constraints_v1_interface.name) == 0) {
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

static void registry_global_remove(void*, wl_registry*, uint32_t) {
}

static void zxdg_decoration_configure(
    void*, zxdg_toplevel_decoration_v1*, uint32_t mode
) {
    serverDecorations = mode == ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE;
    if (!serverDecorations && window != nullptr) {
        window->enableOwnDecorations();
    }
}

std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>>
wayland_window_initialize(DisplaySettings* settings, std::string title) {
    state.display = wl_display_connect(nullptr);
    if (state.display == nullptr) {
        logger.error() << "could not connect to the Wayland display";
        return {nullptr, nullptr};
    }
    static const wl_registry_listener registryListener = [] {
        wl_registry_listener value {};
        value.global = registry_global;
        value.global_remove = registry_global_remove;
        return value;
    }();
    state.registry = wl_display_get_registry(state.display);
    wl_registry_add_listener(state.registry, &registryListener, nullptr);
    wl_display_roundtrip(state.display);
    if (state.compositor == nullptr || state.wmBase == nullptr) {
        logger.error() << "the compositor does not provide xdg-shell";
        wl_display_disconnect(state.display);
        state.display = nullptr;
        return {nullptr, nullptr};
    }

    state.surface = wl_compositor_create_surface(state.compositor);

    auto inputPtr = std::make_unique<WaylandInput>();
    input = inputPtr.get();
    auto windowPtr = std::make_unique<WaylandWindow>(
        *inputPtr, settings, settings->width.get(), settings->height.get()
    );
    window = windowPtr.get();

    static const wl_data_device_listener deviceListener = [] {
        wl_data_device_listener value {};
        value.data_offer = data_device_data_offer;
        value.enter = ignore_event<>;
        value.leave = ignore_event<>;
        value.motion = ignore_event<>;
        value.drop = ignore_event<>;
        value.selection = data_device_selection;
        return value;
    }();
    if (state.dataDeviceManager && state.seat) {
        state.dataDevice = wl_data_device_manager_get_data_device(
            state.dataDeviceManager, state.seat
        );
        wl_data_device_add_listener(state.dataDevice, &deviceListener, nullptr);
    }

    {
        state.xdgSurface = xdg_wm_base_get_xdg_surface(
            state.wmBase, state.surface
        );
        state.toplevel = xdg_surface_get_toplevel(state.xdgSurface);
        window->setTitle(title);
        xdg_toplevel_set_app_id(state.toplevel, "VoxelCore");
        if (state.decorationManager) {
            state.decoration =
                zxdg_decoration_manager_v1_get_toplevel_decoration(
                    state.decorationManager, state.toplevel
                );
            static const zxdg_toplevel_decoration_v1_listener listener = [] {
                zxdg_toplevel_decoration_v1_listener value {};
                value.configure = zxdg_decoration_configure;
                return value;
            }();
            zxdg_toplevel_decoration_v1_add_listener(
                state.decoration, &listener, nullptr
            );
            zxdg_toplevel_decoration_v1_set_mode(
                state.decoration, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE
            );
        }
        if (state.decorationManager == nullptr) {
            window->enableOwnDecorations();
        }
        static const xdg_surface_listener surfaceListener = [] {
            xdg_surface_listener value {};
            value.configure = xdg_surface_configure;
            return value;
        }();
        static const xdg_toplevel_listener toplevelListener = [] {
            xdg_toplevel_listener value {};
            value.configure = toplevel_configure;
            value.close = toplevel_close;
            value.configure_bounds = toplevel_configure_bounds;
            value.wm_capabilities = toplevel_wm_capabilities;
            return value;
        }();
        xdg_surface_add_listener(state.xdgSurface, &surfaceListener, nullptr);
        xdg_toplevel_add_listener(state.toplevel, &toplevelListener, nullptr);
        wl_surface_commit(state.surface);
    }

    while (!window->configured) {
        if (wl_display_dispatch(state.display) == -1) {
            logger.error() << "the Wayland connection was lost";
            return {nullptr, nullptr};
        }
    }

    window->eglDisplay = eglGetPlatformDisplay(
        EGL_PLATFORM_WAYLAND_KHR, state.display, nullptr
    );
    if (window->eglDisplay == EGL_NO_DISPLAY ||
        eglInitialize(window->eglDisplay, nullptr, nullptr) == EGL_FALSE) {
        logger.error() << "could not initialize EGL";
        return {nullptr, nullptr};
    }
    eglBindAPI(EGL_OPENGL_API);

    EGLint samples = settings->samples.get();
    EGLConfig config = nullptr;
    EGLint configCount = 0;
    const EGLint configAttributes[] {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8,
        EGL_SAMPLES, samples > 0 ? samples : 0,
        EGL_NONE
    };
    eglChooseConfig(window->eglDisplay, configAttributes, &config, 1, &configCount);
    if (samples > 0 && configCount == 0) {
        logger.warning() << "multisampling is not available";
        const EGLint fallbackAttributes[] {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8,
            EGL_NONE
        };
        eglChooseConfig(
            window->eglDisplay, fallbackAttributes, &config, 1, &configCount
        );
    }
    if (configCount == 0) {
        logger.error() << "no suitable EGL config";
        return {nullptr, nullptr};
    }

    const EGLint contextAttributes[] {
        EGL_CONTEXT_MAJOR_VERSION, 3,
        EGL_CONTEXT_MINOR_VERSION, 3,
        EGL_NONE
    };
    window->eglContext = eglCreateContext(
        window->eglDisplay, config, EGL_NO_CONTEXT, contextAttributes
    );
    window->eglWindow = wl_egl_window_create(
        state.surface, window->getSize().x, window->getSize().y
    );
    window->eglSurface = eglCreateWindowSurface(
        window->eglDisplay, config,
        reinterpret_cast<EGLNativeWindowType>(window->eglWindow), nullptr
    );
    if (window->eglContext == EGL_NO_CONTEXT ||
        window->eglSurface == EGL_NO_SURFACE ||
        eglMakeCurrent(
            window->eglDisplay, window->eglSurface, window->eglSurface,
            window->eglContext
        ) == EGL_FALSE) {
        logger.error() << "could not create the OpenGL context";
        return {nullptr, nullptr};
    }
    eglSwapInterval(window->eglDisplay, 0);

    glewExperimental = GL_TRUE;
    GLenum glewError = glewInit();
    if (glewError != GLEW_OK && glewError != GLEW_ERROR_NO_GLX_DISPLAY) {
        logger.error() << "failed to initialize GLEW:\n"
                       << glewGetErrorString(glewError);
        return {nullptr, nullptr};
    }

    window->initStretchRenderer();
    glViewport(0, 0, window->getSize().x, window->getSize().y);
    glClearColor(0.0f, 0.0f, 0.0f, 1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLint maxTextureSize[1] {static_cast<GLint>(Texture::MAX_RESOLUTION)};
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, maxTextureSize);
    if (maxTextureSize[0] > 0) {
        Texture::MAX_RESOLUTION = maxTextureSize[0];
        logger.info() << "max texture size is " << Texture::MAX_RESOLUTION;
    }
    logger.info() << "GL Vendor: "
                  << reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    logger.info() << "GL Renderer: "
                  << reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    logger.info() << "windowing platform: Wayland (native)";

    return {std::move(windowPtr), std::move(inputPtr)};
}
