#pragma once

#include <memory>
#include <string>
#include <tuple>

class Window;
class Input;
struct DisplaySettings;

std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>>
glfw_window_initialize(DisplaySettings* settings, std::string title);

#ifdef VOXELCORE_WAYLAND
std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>>
wayland_window_initialize(DisplaySettings* settings, std::string title);
#endif
