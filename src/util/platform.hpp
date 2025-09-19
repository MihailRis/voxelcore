#pragma once

#include <string>
#include <filesystem>

namespace platform {
    void configure_encoding();
    /// @return environment locale in ISO format ll_CC
    std::string detect_locale();
    /// @brief Open folder using system file manager asynchronously
    /// @param folder target folder
    void open_folder(const std::filesystem::path& folder);
    /// Makes the current thread sleep for the specified amount of milliseconds.
    void sleep(size_t millis);
    int get_process_id();
    bool openURL(const std::string& url);
}
