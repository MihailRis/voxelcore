#pragma once

#include "window/detail/FrameLayout.hpp"

struct GLFWwindow;

/// @name Window operations the window manager performs itself
///
/// The frame is drawn by the engine, but moving and resizing are better left
/// to the window manager: it keeps window snapping, tiling and the resize
/// feedback native. Every function returns false when that is not available
/// and the caller has to move or resize the window on its own.
/// @{

/// @brief Whether the window manager can be asked to do it
bool frame_has_native_operations();

/// @brief Ask the window manager to start moving the window
bool frame_start_move(GLFWwindow* window);

/// @brief Ask the window manager to start resizing the window
bool frame_start_resize(GLFWwindow* window, int edges);

/// @brief Hide the caption of the native frame, keeping the rest of it
/// @param hitTest tells what the pointer is over
void frame_setup_native(GLFWwindow* window, frame::Hit (*hitTest)(void*, int, int), void* userdata);

/// @}
