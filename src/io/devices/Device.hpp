#pragma once

#include <string>
#include <memory>
#include <iosfwd>
#include <filesystem>

#include "../path.hpp"

namespace io {
    /// @brief Device interface for file system operations
    class Device {
    public:
        virtual ~Device() = default;

        /// @brief Resolve path to the filesystem path
        virtual std::filesystem::path resolve(std::string_view path) = 0;

        /// @brief Open file for writing
        /// @throw std::runtime_error if file cannot be opened
        virtual std::unique_ptr<std::ostream> write(std::string_view path) = 0;

        /// @brief Open file for reading
        /// @throw std::runtime_error if file cannot be opened
        virtual std::unique_ptr<std::istream> read(std::string_view path) = 0;

        /// @brief Get file size in bytes
        virtual size_t size(std::string_view path) = 0;

        /// @brief Get file last write timestamp 
        virtual file_time_type lastWriteTime(std::string_view path) = 0;

        /// @brief Check if file or directory exists
        virtual bool exists(std::string_view path) = 0;

        /// @brief Check if path is a directory
        virtual bool isdir(std::string_view path) = 0;

        /// @brief Check if path is a regular file
        virtual bool isfile(std::string_view path) = 0;

        /// @brief Create directory
        /// @return true if directory was created
        virtual bool mkdir(std::string_view path) = 0;

        /// @brief Create directories recursively
        /// @return true if directory was created
        virtual bool mkdirs(std::string_view path) = 0;

        /// @brief Remove file or empty directory
        /// @return true if file or directory was removed
        virtual bool remove(std::string_view path) = 0;

        /// @brief Remove all files and directories in the folder recursively
        /// @return number of removed files and directories
        virtual uint64_t removeAll(std::string_view path) = 0;

        /// @brief List directory contents
        virtual std::unique_ptr<PathsGenerator> list(std::string_view path) = 0;
    };

    /// @brief Subdevice is a wrapper around another device limited to a directory
    class SubDevice : public Device {
    public:
        SubDevice(
            std::shared_ptr<Device> parent,
            const std::string& path,
            bool createDirectory = true
        );

        std::filesystem::path resolve(std::string_view path) override {
            return parent->resolve((root / path).pathPart());
        }

        std::unique_ptr<std::ostream> write(std::string_view path) override {
            return parent->write((root / path).pathPart());
        }

        std::unique_ptr<std::istream> read(std::string_view path) override {
            return parent->read((root / path).pathPart());
        }

        size_t size(std::string_view path) override {
            return parent->size((root / path).pathPart());
        }

        file_time_type lastWriteTime(std::string_view path) override {
            return parent->lastWriteTime((root / path).pathPart());
        }

        bool exists(std::string_view path) override {
            return parent->exists((root / path).pathPart());
        }

        bool isdir(std::string_view path) override {
            return parent->isdir((root / path).pathPart());
        }

        bool isfile(std::string_view path) override {
            return parent->isfile((root / path).pathPart());
        }

        bool mkdir(std::string_view path) override {
            return parent->mkdir((root / path).pathPart());
        }

        bool mkdirs(std::string_view path) override {
            return parent->mkdirs((root / path).pathPart());
        }

        bool remove(std::string_view path) override {
            return parent->remove((root / path).pathPart());
        }

        uint64_t removeAll(std::string_view path) override {
            return parent->removeAll((root / path).pathPart());
        }

        std::unique_ptr<PathsGenerator> list(std::string_view path) override {
            return parent->list((root / path).pathPart());
        }
    private:
        std::shared_ptr<Device> parent;
        path root;
    };
}
