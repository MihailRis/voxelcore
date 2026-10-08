#include "Window.hpp"

#include "debug/Logger.hpp"
#include "detail/WindowBackends.hpp"
#include "window/input.hpp"

#include <cstdlib>

static debug::Logger logger("window");

#ifdef VOXELENGINE_WAYLAND
static bool prefer_wayland() {
    const char* display = getenv("WAYLAND_DISPLAY");
    if (display != nullptr && *display) {
        return true;
    }
    return getenv("DISPLAY") == nullptr;
}
#endif

std::tuple<
    std::unique_ptr<Window>,
    std::unique_ptr<Input>
> Window::initialize(DisplaySettings* settings, std::string title) {
    input_util::initialize();
#ifdef VOXELENGINE_WAYLAND
    if (prefer_wayland()) {
        auto result = wayland_window_initialize(settings, title);
        if (std::get<0>(result) && std::get<1>(result)) {
            return result;
        }
        logger.warning() << "could not initialize the Wayland backend";
    }
#endif
    return glfw_window_initialize(settings, std::move(title));
}
