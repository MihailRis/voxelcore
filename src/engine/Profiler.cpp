#include "Profiler.hpp"

#if defined(VOXELENGINE_PROFILER)
#if defined(VOXELENGINE_TRACY_PROFILER)
void* operator new(std::size_t count) {
    auto ptr = malloc(count);
    TracyAlloc(ptr, count);
    return ptr;
}
void operator delete(void* ptr) noexcept {
    TracyFree(ptr);
    free(ptr);
}
#else
#error "No active memory profiler overloading"
#endif
#endif
