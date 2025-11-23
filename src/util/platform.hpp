#pragma once

#include <string>
#include <vector>
#include <filesystem>

namespace platform {
    void configure_encoding();
    /// @brief Get Environment locale in ISO format ll_CC
    std::string detect_locale();
    /// @brief Open folder using system file manager asynchronously
    /// @param folder target folder
    void open_folder(const std::filesystem::path& folder);
    /// @brief Makes the current thread sleep for the specified amount of milliseconds.
    void sleep(size_t millis);
    /// @brief Get current process id 
    int get_process_id();
    /// @brief Get current process running executable path  
    std::filesystem::path get_executable_path();
    /// @brief Run a separate engine instance with specified arguments
    void new_engine_instance(const std::vector<std::string>& args);
    /// @brief Open URL in web browser 
    bool open_url(const std::string& url);
}
