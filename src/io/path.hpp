#pragma once

#include <string>
#include <stdexcept>
#include <filesystem>

namespace io {
    using file_time_type = std::filesystem::file_time_type;
    
    /// @brief Access violation error
    class access_error : public std::runtime_error {
    public:
        access_error(const std::string& msg) : std::runtime_error(msg) {
        }
    };

    /// @brief std::filesystem::path project-specific alternative having
    /// `entry_point:path` scheme and solving std::filesystem::path problems:
    /// - implicit std::string conversions depending on compiler
    /// - unicode path construction must be done with std::filesystem::u8path
    class path {
    public:
        path() = default;

        path(std::string str) : str(std::move(str)) {
            colonPos = this->str.find(':');
            
            size_t len = this->str.length();
            for (size_t i = 0; i < len; i++) {
                if (this->str[i] == '\\') {
                    this->str[i] = '/';
                }
            }
        }

        path(std::string_view str) : path(std::string(str)) {}

        path(const char* str) : path(std::string(str)) {}

        bool operator==(const std::string& other) const {
            return str == other;
        }

        bool operator<(const path& other) const {
            return str < other.str;
        }

        bool operator==(const path& other) const {
            return str == other.str;
        }

        bool operator==(const char* other) const {
            return str == other;
        }

        path operator/(const char* child) const {
            if (str.empty() || str[str.length()-1] == ':') {
                return str + std::string(child);
            }
            return str + "/" + std::string(child);
        }

        path operator/(const std::string& child) const {
            if (str.empty() || str[str.length()-1] == ':') {
                return str + child;
            }
            return str + "/" + child;
        }

        path operator/(std::string_view child) const {
            if (str.empty() || str[str.length()-1] == ':') {
                return str + std::string(child);
            }
            return str + "/" + std::string(child);
        }

        path operator/(const path& child) const {
            if (str.empty() || str[str.length()-1] == ':') {
                return str + child.pathPart();
            }
            return str + "/" + child.pathPart();
        }

        std::string pathPart() const {
            if (colonPos == std::string::npos) {
                return str;
            }
            return str.substr(colonPos + 1);
        }

        std::string name() const {
            size_t slashpos = str.rfind('/');
            if (slashpos == std::string::npos) {
                return colonPos == std::string::npos ? str
                                                     : str.substr(colonPos + 1);
            }
            return str.substr(slashpos + 1);
        }

        std::string stem() const {
            return name().substr(0, name().rfind('.'));
        }

        /// @brief Get extension
        std::string extension() const {
            size_t slashpos = str.rfind('/');
            size_t dotpos = str.rfind('.');
            if (dotpos == std::string::npos ||
                (slashpos != std::string::npos && dotpos < slashpos)) {
                return "";
            }
            return str.substr(dotpos);
        }

        /// @brief Get entry point
        std::string entryPoint() const {
            checkValid();
            return str.substr(0, colonPos);
        }

        /// @brief Get parent path
        path parent() const;

        path normalized() const;

        std::string string() const {
            return str;
        }

        /// @brief Check if path is not initialized with 'entry_point:path'
        bool empty() const {
            return str.empty();
        }

        bool emptyOrInvalid() const {
            return str.empty() || colonPos == std::string::npos;
        }
    private:
        /// @brief UTF-8 string contains entry_point:path or empty string
        std::string str;
        /// @brief Precalculated position of colon character
        size_t colonPos = std::string::npos;

        void checkValid() const;
    };

    class PathsGenerator {
    public:
        virtual ~PathsGenerator() = default;
        virtual bool next(path& dst) = 0;
    };
}
