project(VoxelCoreSrc)

file(GLOB_RECURSE headers ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# The window backends are separate libraries: the engine only talks to the
# Window/Input interfaces and to the factories in detail/WindowBackends.hpp
file(GLOB_RECURSE backend_sources
     ${CMAKE_CURRENT_SOURCE_DIR}/window/detail/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/window/wayland/*.cpp)
list(REMOVE_ITEM sources ${backend_sources})

add_library(VoxelCoreSrc STATIC ${sources} ${headers})

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

target_include_directories(VoxelCoreSrc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set(VOXEL_COMPILE_OPTIONS
    $<$<CXX_COMPILER_ID:MSVC>:
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

# Everything the engine sources and their public headers include: the backends
# compile against the same surface, and a missing include directory shows up as
# a build failure only on the platforms that keep the library outside the
# default include path.
add_library(VoxelCoreInterface INTERFACE)

target_include_directories(VoxelCoreInterface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(VoxelCoreInterface INTERFACE
    glfw
    GLEW::GLEW
    OpenGL::GL
    glm::glm)

option(VOXELCORE_WAYLAND "Build the native Wayland window backend" ON)

add_subdirectory(window/detail)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND VOXELCORE_WAYLAND)
    add_subdirectory(window/wayland)
endif()

target_link_libraries(
    VoxelCoreSrc
    PRIVATE VoxelGLFW
            glfw
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

if(TARGET VoxelWayland)
    target_link_libraries(VoxelCoreSrc PRIVATE VoxelWayland)
endif()

target_compile_options(VoxelCoreSrc PUBLIC ${VOXEL_COMPILE_OPTIONS})

target_link_options(
    VoxelCoreSrc
    PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:
    -no-pie
    >
    # Need for static compilation on Windows with clang TODO: Make single build
    # on Windows to avoid dependence on combinations of platforms and compilers
    # and make it independent
    $<$<PLATFORM_ID:Windows>:$<$<CXX_COMPILER_ID:Clang>:-static>>)
