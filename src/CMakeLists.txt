project(VoxelEngineSrc)

file(GLOB_RECURSE headers ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

add_library(VoxelEngineSrc STATIC ${sources} ${headers})

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # specific for vcpkg
    find_package(OpenAL CONFIG REQUIRED)
else()
    find_package(OpenAL REQUIRED)
endif()
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenSSL 3.0 REQUIRED COMPONENTS Crypto)
find_package(glfw3 REQUIRED)
find_package(Freetype REQUIRED)
if(NOT APPLE)
    find_package(EnTT REQUIRED)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    find_package(vorbis REQUIRED)
    
    set(LUAJIT_PACKAGE_DIR "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
    if(VCPKG_TARGET_TRIPLET MATCHES "static")
        add_library(luajit STATIC IMPORTED)
        set_target_properties(
            luajit
            PROPERTIES
                IMPORTED_LOCATION
                "${LUAJIT_PACKAGE_DIR}/lib/libluajit-5.1.a"
                INTERFACE_INCLUDE_DIRECTORIES
                "${LUAJIT_PACKAGE_DIR}/include/"
        )
    else()
        if(EXISTS "${LUAJIT_PACKAGE_DIR}/include/luajit-2.1")
            set(LUAJIT_INCLUDE_DIR "${LUAJIT_PACKAGE_DIR}/include/luajit-2.1")
        else()
            set(LUAJIT_INCLUDE_DIR "${LUAJIT_PACKAGE_DIR}/include/luajit")
        endif()

        add_library(luajit SHARED IMPORTED)
        set_target_properties(
            luajit
            PROPERTIES
                IMPORTED_LOCATION
                "${LUAJIT_PACKAGE_DIR}/bin/lua51.dll"
                IMPORTED_IMPLIB
                "${LUAJIT_PACKAGE_DIR}/lib/lua51.lib"
                INTERFACE_INCLUDE_DIRECTORIES "${LUAJIT_INCLUDE_DIR}"
        )
    endif()

    add_library(luajit::luajit ALIAS luajit)
else()
    find_package(PkgConfig)

    pkg_check_modules(luajit REQUIRED IMPORTED_TARGET luajit)
    pkg_check_modules(vorbis REQUIRED IMPORTED_TARGET vorbis)
    pkg_check_modules(vorbisfile REQUIRED IMPORTED_TARGET vorbisfile)
    add_library(Vorbis::vorbis ALIAS PkgConfig::vorbis)
    add_library(Vorbis::vorbisfile ALIAS PkgConfig::vorbisfile)
    add_library(luajit::luajit ALIAS PkgConfig::luajit)
endif()

option(VOXELENGINE_WAYLAND "Build the native Wayland window backend" ON)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND VOXELENGINE_WAYLAND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(wayland-client IMPORTED_TARGET wayland-client)
    pkg_check_modules(xkbcommon IMPORTED_TARGET xkbcommon)
    pkg_check_modules(egl IMPORTED_TARGET egl)
    find_library(WAYLAND_EGL_LIBRARY NAMES wayland-egl libwayland-egl.so.1)
    find_program(WAYLAND_SCANNER wayland-scanner)

    if(NOT wayland-client_FOUND OR NOT xkbcommon_FOUND OR NOT egl_FOUND
       OR NOT WAYLAND_EGL_LIBRARY OR NOT WAYLAND_SCANNER)
        message(STATUS "Wayland backend is off: dependencies are not found")
    else()
        set(WAYLAND_PROTOCOLS ${CMAKE_CURRENT_SOURCE_DIR}/window/wayland/protocols)
        set(WAYLAND_GENERATED ${CMAKE_CURRENT_BINARY_DIR}/wayland-protocols)
        file(MAKE_DIRECTORY ${WAYLAND_GENERATED})
        set(WAYLAND_SOURCES)

        function(voxel_wayland_protocol XML NAME)
            add_custom_command(
                OUTPUT ${WAYLAND_GENERATED}/${NAME}-client-protocol.h
                       ${WAYLAND_GENERATED}/${NAME}-protocol.c
                COMMAND ${WAYLAND_SCANNER} client-header
                        ${WAYLAND_PROTOCOLS}/${XML}
                        ${WAYLAND_GENERATED}/${NAME}-client-protocol.h
                COMMAND ${WAYLAND_SCANNER} private-code
                        ${WAYLAND_PROTOCOLS}/${XML}
                        ${WAYLAND_GENERATED}/${NAME}-protocol.c
                DEPENDS ${WAYLAND_PROTOCOLS}/${XML}
                VERBATIM)
            set(WAYLAND_SOURCES ${WAYLAND_SOURCES}
                                ${WAYLAND_GENERATED}/${NAME}-protocol.c
                PARENT_SCOPE)
        endfunction()

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

        target_sources(VoxelEngineSrc PRIVATE ${WAYLAND_SOURCES})
        target_include_directories(VoxelEngineSrc PUBLIC ${WAYLAND_GENERATED})
        target_compile_definitions(VoxelEngineSrc PUBLIC VOXELENGINE_WAYLAND=1)
        target_link_libraries(
            VoxelEngineSrc
            PRIVATE PkgConfig::wayland-client
                    PkgConfig::xkbcommon
                    PkgConfig::egl
                    ${WAYLAND_EGL_LIBRARY})
        message(STATUS "Wayland backend is on")
    endif()
endif()

target_include_directories(VoxelEngineSrc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
    VoxelEngineSrc
    PRIVATE glfw
            OpenGL::GL
            GLEW::GLEW
            ZLIB::ZLIB
            PNG::PNG
            CURL::libcurl
            OpenSSL::Crypto
            OpenAL::OpenAL
            Vorbis::vorbis
            Vorbis::vorbisfile
            Freetype::Freetype
    PUBLIC luajit::luajit
           glm::glm # Need public for src/delegates.hpp, which including to
                    # main.cpp
)

target_compile_options(
    VoxelEngineSrc
    PUBLIC $<$<CXX_COMPILER_ID:MSVC>:
           /utf-8
           /MP
           /D_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR
           /W4
           /wd4244 # conversion from 'a' to 'T', possible loss of data
           /wd4267 # conversion from 'size_t' to 'int', possible loss of data
           /wd4245 # conversion from 'int' to 'const size_t', signed/unsigned
                   # mismatch
           /wd4100 # unreferenced formal parameter
           /wd4457 # declaration of 'var' hides function parameter 
           /wd4458 # declaration of 'var' hides class member
           /wd4459 # declaration of 'var' hides global declaration 
           /wd4101 # 'var': unreferenced local variable
           /wd4388 # 'token' : signed/unsigned mismatch
           /wd4018 # '>': signed/unsigned mismatch
           >
           $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:
           -Wall
           -Wextra
           # additional warnings
           -Wformat-nonliteral
           #-Wsign-conversion
           -Wcast-align
           -Wpointer-arith
           -Wundef
           -Wwrite-strings
           -Wno-unused-parameter
           -Wno-sign-compare
           -Wno-unknown-pragmas
           >
           $<$<CXX_COMPILER_ID:GNU>:
           -Wduplicated-branches
           -Wduplicated-cond
           >)

target_link_options(
    VoxelEngineSrc
    PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:
    -no-pie
    >
    # Need for static compilation on Windows with clang TODO: Make single build
    # on Windows to avoid dependence on combinations of platforms and compilers
    # and make it independent
    $<$<PLATFORM_ID:Windows>:$<$<CXX_COMPILER_ID:Clang>:-static>>)
