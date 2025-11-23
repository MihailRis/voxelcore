#include "MemoryDevice.hpp"

#include "../memory_istream.hpp"
#include "../memory_ostream.hpp"
#include "../finalizing_ostream.hpp"

#include <algorithm>

io::MemoryDevice::MemoryDevice() {}

std::filesystem::path io::MemoryDevice::resolve(std::string_view path) {
    throw std::runtime_error("unable to resolve filesystem path");
}

std::unique_ptr<std::ostream> io::MemoryDevice::write(std::string_view path) {
    std::string filePath = std::string(path);
    return std::make_unique<finalizing_ostream>(
        std::make_unique<memory_ostream>(),
        [this, filePath](auto ostream) {
            auto& memoryStream = dynamic_cast<memory_ostream&>(*ostream);
            createFile(std::move(filePath), memoryStream.release());
        }
    );
}

std::unique_ptr<std::istream> io::MemoryDevice::read(std::string_view path) {
    const auto& found = nodes.find(std::string(path));
    if (found == nodes.end()) {
        return nullptr;
    }
    auto& node = found->second;
    if (auto file = node.get_if<File>()) {
        if (file->content != nullptr) {
            return std::make_unique<memory_view_istream>(file->content);
        }
    }
    return nullptr;
}

size_t io::MemoryDevice::size(std::string_view path) {
    const auto& found = nodes.find(std::string(path));
    if (found == nodes.end()) {
        return 0;
    }
    return std::visit([](auto&& arg) -> size_t {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, File>) {
            return arg.content.size();
        } else if constexpr (std::is_same_v<T, Dir>) {
            return arg.content.size();
        } else {
            return 0;
        }
    }, found->second.data);
}

io::file_time_type io::MemoryDevice::lastWriteTime(std::string_view path) {
    return file_time_type::min();
}

bool io::MemoryDevice::exists(std::string_view path) {
    if (path.empty()) {
        return true;
    }
    return nodes.find(std::string(path)) != nodes.end();
}

bool io::MemoryDevice::isdir(std::string_view path) {
    if (path.empty()) {
        return true;
    }
    const auto& found = nodes.find(std::string(path));
    if (found == nodes.end()) {
        return false;
    }
    return found->second.holds_alternative<Dir>();
}

bool io::MemoryDevice::isfile(std::string_view path)  {
    const auto& found = nodes.find(std::string(path));
    if (found == nodes.end()) {
        return false;
    }
    return found->second.holds_alternative<File>();
}

bool io::MemoryDevice::mkdir(std::string_view path) {
    return createDir(std::string(path)) != nullptr;
}

bool io::MemoryDevice::mkdirs(std::string_view path) {
    io::path dirPath = std::string(path);
    std::vector<std::string> parts;
    while (!dirPath.pathPart().empty()) {
        parts.push_back(dirPath.name());
        dirPath = dirPath.parent();
    }
    for (int i = parts.size() - 1; i >= 0; i--) {
        dirPath = dirPath / parts[i];
        createDir(dirPath.string());
    }
    return true;
}

bool io::MemoryDevice::remove(std::string_view path) {
    std::string pathString = std::string(path);
    const auto& found = nodes.find(pathString);
    if (found == nodes.end()) {
        return false;
    }
    if (found->second.holds_alternative<Dir>()) {
        const auto& dir = found->second.get_if<Dir>();
        if (!dir->content.empty()) {
            return false;
        }
    }
    io::path filePath = pathString;
    io::path parentPath = filePath.parent();
    auto parentDir = getDir(parentPath.string());
    if (parentDir) {
        auto& content = parentDir->content;
        content.erase(
            std::remove(content.begin(), content.end(), filePath.name()),
            content.end()
        );
    }
    nodes.erase(found);
    return true;
}

uint64_t io::MemoryDevice::removeAll(std::string_view path) {
    std::string pathString = std::string(path);
    const auto& found = nodes.find(pathString);
    if (found == nodes.end()) {
        return 0;
    }
    io::path filePath = pathString;

    uint64_t count = 0;
    if (found->second.holds_alternative<Dir>()) {
        auto dir = found->second.get_if<Dir>();
        auto files = dir->content;
        for (const auto& name : files) {
            io::path subPath = filePath / name;
            count += removeAll(subPath.string());
        }
    }
    if (remove(pathString)) {
        count++;
    }
    return count;
}

namespace {
    struct MemoryPathsGenerator : public io::PathsGenerator {
        std::vector<std::string> entries;
        size_t index = 0;

        MemoryPathsGenerator(std::vector<std::string>&& entries)
            : entries(std::move(entries)) {}

        bool next(io::path& outPath) override {
            if (index >= entries.size()) {
                return false;
            }
            outPath = entries[index++];
            return true;
        }
    };
}

std::unique_ptr<io::PathsGenerator> io::MemoryDevice::list(std::string_view path) {
    auto dir = getDir(path);
    if (!dir) {
        return nullptr;
    }
    return std::make_unique<MemoryPathsGenerator>(
        std::vector<std::string>(dir->content)
    );
}

io::MemoryDevice::Dir* io::MemoryDevice::createDir(std::string path) {
    io::path filePath = path;
    io::path parent = filePath.parent();
    auto parentDir = getDir(parent.string());
    if (!parentDir) {
        return nullptr;
    }
    parentDir->content.push_back(filePath.name());
    auto& node = nodes[std::move(path)];
    node.data = Dir {};
    return node.get_if<Dir>();
}

io::MemoryDevice::Node* io::MemoryDevice::createFile(
    std::string path, util::Buffer<char>&& content
) {
    io::path filePath = path;
    io::path parent = filePath.parent();
    auto dir = getDir(parent.string());
    if (!dir) {
        return nullptr;
    }
    dir->content.push_back(filePath.name());
    auto& node = nodes[std::move(path)];
    node.data = File {std::move(content)};
    return &node;
}

io::MemoryDevice::Dir* io::MemoryDevice::getDir(std::string_view path) {
    if (path.empty()) {
        return &rootDir;
    }
    const auto& found = nodes.find(std::string(path));
    if (found == nodes.end()) {
        return nullptr;
    }
    auto& node = found->second;
    return node.get_if<Dir>();
}
