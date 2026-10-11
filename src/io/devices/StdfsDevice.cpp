#include "StdfsDevice.hpp"

#include <fstream>
#include <filesystem>

#include "debug/Logger.hpp"

using namespace io;
namespace fs = std::filesystem;

static debug::Logger logger("io-stdfs");

StdfsDevice::StdfsDevice(fs::path root, bool createDirectory)
 : root(std::move(root)) {
    if (createDirectory && !fs::is_directory(this->root)) {
        std::error_code ec;
        fs::create_directories(this->root, ec);
        if (ec) {
            logger.error() << "error creating root directory " << this->root
                           << ": " << ec.message();
        }
    }
    absoluteRoot = fs::absolute(this->root).lexically_normal();
    if (!absoluteRoot.has_filename() && absoluteRoot.has_relative_path()) {
        absoluteRoot = absoluteRoot.parent_path();
    }
}

fs::path StdfsDevice::resolve(std::string_view path) {
    if (path.find_first_of(std::string_view("\0\\:", 3)) != std::string_view::npos) {
        throw std::runtime_error("invalid path");
    }
    fs::path rel = fs::u8path(io::path(std::string(path)).normalized().string());
    if (rel.has_root_name() || rel.has_root_directory()) {
        throw std::runtime_error("absolute paths are not allowed");
    }
    fs::path full = (absoluteRoot / rel).lexically_normal();
    fs::path inside = full.lexically_relative(absoluteRoot);
    if (inside.empty() || *inside.begin() == "..") {
        throw std::runtime_error("path escapes device root");
    }
    if (inside == ".") {
        return absoluteRoot;
    }
    return full;
}

std::unique_ptr<std::ostream> StdfsDevice::write(std::string_view path) {
    auto resolved = resolve(path);
    auto output = std::make_unique<std::ofstream>(resolved, std::ios::binary);
    if (!output->is_open()) {
        throw std::runtime_error("could not to open file " + resolved.u8string());
    }
    return output;
}

std::unique_ptr<std::istream> StdfsDevice::read(std::string_view path) {
    auto resolved = resolve(path);
    auto input = std::make_unique<std::ifstream>(resolved, std::ios::binary);
    if (!*input) {
        throw std::runtime_error("could not to open file " + resolved.u8string());
    }
    return input;
}

size_t StdfsDevice::size(std::string_view path) {
    return fs::file_size(resolve(path));
}

file_time_type StdfsDevice::lastWriteTime(std::string_view path) {
    return fs::last_write_time(resolve(path));
}

bool StdfsDevice::exists(std::string_view path) {
    return fs::exists(resolve(path));
}

bool StdfsDevice::isdir(std::string_view path) {
    return fs::is_directory(resolve(path));
}

bool StdfsDevice::isfile(std::string_view path) {
    return fs::is_regular_file(resolve(path));
}

bool StdfsDevice::mkdir(std::string_view path) {
    auto resolved = resolve(path);

    std::error_code ec;
    bool created = fs::create_directory(resolved, ec);
    if (ec) {
        logger.error() << "error creating directory " << resolved << ": "
                       << ec.message();
    }
    return created;
}

bool StdfsDevice::mkdirs(std::string_view path) {
    auto resolved = resolve(path);

    std::error_code ec;
    bool created = fs::create_directories(resolved, ec);
    if (ec) {
        logger.error() << "error creating directories " << resolved << ": "
                       << ec.message();
    }
    return created;
}

bool StdfsDevice::remove(std::string_view path) {
    auto resolved = resolve(path);
    if (resolved == absoluteRoot) {
        throw std::runtime_error("unable to remove device root");
    }
    std::error_code ec;
    bool removed = fs::remove(resolved, ec);
    if (ec) {
        logger.error() << "error removing " << resolved << ": " << ec.message();
    }
    return removed;
}

uint64_t StdfsDevice::removeAll(std::string_view path) {
    auto resolved = resolve(path);
    if (resolved == absoluteRoot) {
        throw std::runtime_error("unable to remove device root");
    }
    std::error_code ec;
    auto removed = fs::remove_all(resolved, ec);
    if (ec) {
        logger.error() << "error removing " << resolved << ": " << ec.message();
        return 0;
    }
    if (removed > 0) {
        logger.info() << "removeAll " << resolved << ": " << removed;
    }
    return removed;
}

class StdfsPathsGenerator : public PathsGenerator {
public:
    StdfsPathsGenerator(fs::path root) : root(std::move(root)) {
        it = fs::directory_iterator(this->root);
    }

    bool next(io::path& path) override {
        if (it == fs::directory_iterator()) {
            return false;
        }
        path = it->path().filename().u8string();
        it++;
        return true;
    }
private:
    fs::path root;
    fs::directory_iterator it;
};

std::unique_ptr<PathsGenerator> StdfsDevice::list(std::string_view path) {
    return std::make_unique<StdfsPathsGenerator>(resolve(path));
}
