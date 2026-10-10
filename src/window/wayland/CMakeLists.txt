project(VoxelWayland)

# The library is linked from the parent directory, so the imported targets of
# its dependencies have to be visible outside of this directory too.
function(voxel_wayland_pkg_config MODULE)
    pkg_check_modules(${MODULE} IMPORTED_TARGET ${ARGN})
    if(TARGET PkgConfig::${MODULE})
        set_target_properties(
            PkgConfig::${MODULE} PROPERTIES IMPORTED_GLOBAL TRUE)
    endif()
endfunction()

macro(voxel_wayland_protocol XML NAME)
    add_custom_command(
        OUTPUT ${WAYLAND_GENERATED}/${NAME}-client-protocol.h
               ${WAYLAND_GENERATED}/${NAME}-protocol.c
        COMMAND ${WAYLAND_SCANNER} client-header ${WAYLAND_PROTOCOLS}/${XML}
                ${WAYLAND_GENERATED}/${NAME}-client-protocol.h
        COMMAND ${WAYLAND_SCANNER} private-code ${WAYLAND_PROTOCOLS}/${XML}
                ${WAYLAND_GENERATED}/${NAME}-protocol.c
        DEPENDS ${WAYLAND_PROTOCOLS}/${XML}
        VERBATIM)
    list(APPEND WAYLAND_SOURCES ${WAYLAND_GENERATED}/${NAME}-protocol.c)
endmacro()

function(voxel_add_wayland_backend)
    find_package(PkgConfig REQUIRED)
    voxel_wayland_pkg_config(wayland-client wayland-client)
    voxel_wayland_pkg_config(xkbcommon xkbcommon)
    voxel_wayland_pkg_config(egl egl)
    voxel_wayland_pkg_config(libsystemd libsystemd)
    find_library(WAYLAND_EGL_LIBRARY NAMES wayland-egl libwayland-egl.so.1)
    find_program(WAYLAND_SCANNER wayland-scanner)

    if(NOT wayland-client_FOUND OR NOT xkbcommon_FOUND OR NOT egl_FOUND
       OR NOT WAYLAND_EGL_LIBRARY OR NOT WAYLAND_SCANNER)
        message(STATUS "Wayland backend is off: dependencies are not found")
        return()
    endif()

    set(WAYLAND_PROTOCOLS ${CMAKE_CURRENT_SOURCE_DIR}/protocols)
    set(WAYLAND_GENERATED ${CMAKE_CURRENT_BINARY_DIR}/protocols)
    file(MAKE_DIRECTORY ${WAYLAND_GENERATED})
    set(WAYLAND_SOURCES)

    voxel_wayland_protocol(xdg-shell.xml xdg-shell)
    voxel_wayland_protocol(cursor-shape-v1.xml cursor-shape-v1)
    voxel_wayland_protocol(idle-inhibit-unstable-v1.xml
                           idle-inhibit-unstable-v1)
    voxel_wayland_protocol(pointer-constraints-unstable-v1.xml
                           pointer-constraints-unstable-v1)
    voxel_wayland_protocol(relative-pointer-unstable-v1.xml
                           relative-pointer-unstable-v1)
    voxel_wayland_protocol(xdg-decoration-unstable-v1.xml
                           xdg-decoration-unstable-v1)
    # tablet is referenced by cursor-shape-v1
    voxel_wayland_protocol(tablet-unstable-v2.xml tablet-unstable-v2)

    file(GLOB wayland_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

    add_library(VoxelWayland STATIC ${wayland_sources} ${WAYLAND_SOURCES})

    target_include_directories(VoxelWayland PRIVATE ${WAYLAND_GENERATED})

    target_compile_options(VoxelWayland PRIVATE ${VOXEL_COMPILE_OPTIONS})

    # Linking this library is what enables the backend in the engine code.
    target_compile_definitions(VoxelWayland PUBLIC VOXELCORE_WAYLAND=1)

    target_link_libraries(
        VoxelWayland
        PRIVATE VoxelWindowBase
                VoxelCoreInterface
                PkgConfig::wayland-client
                PkgConfig::xkbcommon
                PkgConfig::egl
                ${WAYLAND_EGL_LIBRARY})

    if(libsystemd_FOUND)
        target_compile_definitions(VoxelWayland PRIVATE VOXELCORE_SDBUS=1)
        target_link_libraries(VoxelWayland PRIVATE PkgConfig::libsystemd)
        message(STATUS "Wayland backend: sd-bus settings are on")
    else()
        message(STATUS "Wayland backend: sd-bus settings are off")
    endif()

    message(STATUS "Wayland backend is on")
endfunction()

voxel_add_wayland_backend()
