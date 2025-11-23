#include <filesystem>

#ifndef _WIN32
#include <unistd.h>
#endif

#ifdef __APPLE__
#include <mach-o/dyld.h> 
#endif

namespace platform::internal {
    std::filesystem::path get_executable_path() {
#ifdef __APPLE__
        char buffer[1024];
        uint32_t size = sizeof(buffer);
        if (_NSGetExecutablePath(buffer, &size) == 0) {
            return std::filesystem::canonical(std::filesystem::path(buffer));
        } else {
            return std::filesystem::path();
        }
#endif
        return std::filesystem::path();
    }
}
