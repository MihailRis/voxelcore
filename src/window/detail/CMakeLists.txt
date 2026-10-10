project(VoxelWindowBackends)

# Input state shared by the window backends.
add_library(VoxelWindowBase STATIC BaseInput.cpp)

target_compile_options(VoxelWindowBase PRIVATE ${VOXEL_COMPILE_OPTIONS})

target_link_libraries(VoxelWindowBase PRIVATE VoxelCoreInterface)

add_library(VoxelGLFW STATIC GLFWWindow.cpp FrameOperations.cpp)

target_compile_options(VoxelGLFW PRIVATE ${VOXEL_COMPILE_OPTIONS})

target_link_libraries(
    VoxelGLFW PRIVATE VoxelWindowBase VoxelCoreInterface ${CMAKE_DL_LIBS})

# On X11 the window manager moves and resizes the window when the engine draws
# the frame. A wayland-only build of GLFW does not answer, and the frame then
# falls back to moving the window itself.
if(UNIX AND NOT APPLE)
    find_package(X11)
    if(X11_FOUND)
        target_compile_definitions(VoxelGLFW PRIVATE VOXEL_X11_FRAME=1)
        target_include_directories(VoxelGLFW PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(VoxelGLFW PRIVATE ${X11_LIBRARIES})
        message(STATUS "window frame: window manager operations are on")
    else()
        message(STATUS "window frame: window manager operations are off")
    endif()
endif()
