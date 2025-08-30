#pragma once

#if defined(VOXELENGINE_PROFILER_GPU)
#if defined(VOXELENGINE_TRACY_PROFILER)
// clang-format off
#include <GL/glew.h>  // Needed by TracyOpenGL.hpp

#include <tracy/TracyOpenGL.hpp>
// clang-format on

#define VOXELENGINE_PROFILE_GPU_CONTEXT TracyGpuContext
#define VOXELENGINE_PROFILE_GPU_COLLECT TracyGpuCollect
#define VOXELENGINE_PROFILE_GPU(name) TracyGpuZone(name)
#else
#error "No active profiler"
#endif
// You can implement any your profiler with adding VOXELENGINE_{NAME}_PROFILER
// macros
#else
#define VOXELENGINE_PROFILE_GPU_CONTEXT
#define VOXELENGINE_PROFILE_GPU_COLLECT
#define VOXELENGINE_PROFILE_GPU(name)
#endif