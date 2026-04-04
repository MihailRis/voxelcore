#pragma once

#include "Device.hpp"
#include "util/Buffer.hpp"

#include <vector>
#include <variant>
#include <optional>
#include <unordered_map>

namespace io {
    /// @brief In-memory filesystem device
    class MemoryDevice : public Device {
        enum class NodeType {
            DIR, FILE
        };

        struct File {
            util::Buffer<char> content;
        };

        struct Dir {
            std::vector<std::string> content;
        };

        struct Node {
            std::variant<Dir, File> data;

            NodeType type() const {
                return std::visit([](auto&& arg) -> NodeType {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, Dir>) return NodeType::DIR;
                    else if constexpr (std::is_same_v<T, File>) return NodeType::FILE;
                }, data);
            }

            template <typename T>
            bool holds_alternative() const {
                return std::holds_alternative<T>(data);
            }

            template <typename T>
            T* get_if() {
                return std::get_if<T>(&data);
            }
        };
    public:
        MemoryDevice();

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
        std::unordered_map<std::string, Node> nodes;
        Dir rootDir {};

        Node* createFile(std::string path, util::Buffer<char>&& content);
        Dir* createDir(std::string path);
        Dir* getDir(std::string_view path);
    };
}
