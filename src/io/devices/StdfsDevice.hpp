#pragma once

#include "Device.hpp"

namespace io {
    /// @brief Device based on the standard filesystem
    class StdfsDevice : public Device {
    public:
        StdfsDevice(std::filesystem::path root, bool createDirectory = true);

        std::filesystem::path resolve(std::string_view path) override;
        std::unique_ptr<std::ostream> write(std::string_view path) override;
        std::unique_ptr<std::istream> read(std::string_view path) override;
        size_t size(std::string_view path) override;
        file_time_type lastWriteTime(std::string_view path) override;
        bool exists(std::string_view path) override;
        bool isdir(std::string_view path) override;
        bool isfile(std::string_view path) override;
        bool mkdir(std::string_view path) override;
        bool mkdirs(std::string_view path) override;
        bool remove(std::string_view path) override;
        uint64_t removeAll(std::string_view path) override;
        std::unique_ptr<PathsGenerator> list(std::string_view path) override;
    private:
        std::filesystem::path root;
    };
}
