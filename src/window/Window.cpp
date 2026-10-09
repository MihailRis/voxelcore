#include "Window.hpp"

#include "debug/Logger.hpp"
#include "detail/WindowBackends.hpp"
#include "window/input.hpp"

#include <cstdlib>

#ifndef GLEW_STATIC
#define GLEW_STATIC
#endif

#include <GL/glew.h>

#include <algorithm>

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

void Window::pushScissor(glm::vec4 area) {
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

void Window::popScissor() {
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

void Window::resetScissor() {
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
    scissorStack = std::stack<glm::vec4>();
    glDisable(GL_SCISSOR_TEST);
}
