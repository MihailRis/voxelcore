#include "EnginePaths.hpp"

#include "debug/Logger.hpp"
#include "io/devices/StdfsDevice.hpp"
#include "io/devices/MemoryDevice.hpp"
#include "io/devices/ZipFileDevice.hpp"
#include "maths/util.hpp"
#include "typedefs.hpp"
#include "util/platform.hpp"
#include "util/random.hpp"
#include "util/stringutil.hpp"
#include "world/files/WorldFiles.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <utility>

namespace fs = std::filesystem;

static std::random_device random_device;

static inline io::path SCREENSHOTS_FOLDER = "user:screenshots";
static inline io::path CONTENT_FOLDER = "user:content";
static inline io::path WORLDS_FOLDER = "user:worlds";

static debug::Logger logger("engine-paths");

template<int n>
static std::string generate_random_base64() {
    auto randomEngine = util::seeded_random_engine(random_device);
    static std::uniform_int_distribution<integer_t> dist(0, 0xFF);
    ubyte bytes[n];
    for (size_t i = 0; i < n; i++) {
        bytes[i] = dist(randomEngine);
    }
    return util::base64_urlsafe_encode(bytes, n);
}

EnginePaths::EnginePaths(CoreParameters& params)
    : resourcesFolder(params.resFolder),
      userFilesFolder(params.userFolder),
      projectFolder(params.projectFolder) {
    if (!params.scriptFile.empty()) {
        scriptFolder = params.scriptFile.parent_path();
        io::set_device("script", std::make_shared<io::StdfsDevice>(*scriptFolder));
    }

    io::set_device("res", std::make_shared<io::StdfsDevice>(resourcesFolder, false));
    io::set_device("user", std::make_shared<io::StdfsDevice>(userFilesFolder));
    io::set_device("project", std::make_shared<io::StdfsDevice>(projectFolder));

    if (!io::is_directory("res:")) {
        throw std::runtime_error(
            resourcesFolder.string() + " is not a directory"
        );
    }
    logger.info() << "executable path: " << platform::get_executable_path().string();
    logger.info() << "resources folder: " << fs::canonical(resourcesFolder).u8string();
    logger.info() << "user files folder: " << fs::canonical(userFilesFolder).u8string();
    logger.info() << "project folder: " << fs::canonical(projectFolder).u8string();
    
    if (!io::is_directory(CONTENT_FOLDER)) {
        io::create_directories(CONTENT_FOLDER);
    }

    io::create_subdevice("core", "res", "");
    io::create_subdevice("export", "user", "export");
    io::create_subdevice("config", "user", "config");
}

std::filesystem::path EnginePaths::getResourcesFolder() const {
    return resourcesFolder;
}

std::filesystem::path EnginePaths::getUserFilesFolder() const {
    return userFilesFolder;
}

io::path EnginePaths::getNewScreenshotFile(const std::string& ext) const {
    auto folder = SCREENSHOTS_FOLDER;
    if (!io::is_directory(folder)) {
        io::create_directories(folder);
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    const char* format = "%Y-%m-%d_%H-%M-%S";
    std::stringstream ss;
    ss << std::put_time(&tm, format);
    std::string datetimestr = ss.str();

    auto file = folder / ("screenshot-" + datetimestr + "." + ext);
    uint index = 0;
    while (io::exists(file)) {
        file = folder / ("screenshot-" + datetimestr + "-" +
                         std::to_string(index) + "." + ext);
        index++;
    }
    return file;
}

io::path EnginePaths::getWorldsFolder() const {
    return WORLDS_FOLDER;
}

io::path EnginePaths::getWorldFolderByName(const std::string& name) {
    return getWorldsFolder() / name;
}

std::vector<io::path> EnginePaths::scanForWorlds() const {
    std::vector<io::path> folders;

    auto folder = getWorldsFolder();
    if (!io::is_directory(folder)) return folders;

    for (const auto& worldFolder : io::directory_iterator(folder)) {
        if (!io::is_directory(worldFolder)) {
            continue;
        }
        auto worldFile = worldFolder / WorldFiles::WORLD_FILE;
        if (!io::is_regular_file(worldFile)) {
            continue;
        }
        folders.push_back(worldFolder);
    }
    std::sort(
        folders.begin(),
        folders.end(),
        [](io::path a, io::path b) {
            a = a / WorldFiles::WORLD_FILE;
            b = b / WorldFiles::WORLD_FILE;
            return fs::last_write_time(io::resolve(a)) >
                   fs::last_write_time(io::resolve(b));
        }
    );
    return folders;
}

void EnginePaths::setCurrentWorldFolder(io::path folder) {
    if (folder.empty()) {
        io::remove_device("world");
    } else {
        io::create_subdevice("world", "user", folder);
    }
    this->currentWorldFolder = std::move(folder);
}

std::string EnginePaths::mount(const io::path& file) {
    if (file.extension() == ".zip") {
        auto stream = io::read(file);
        auto device = std::make_unique<io::ZipFileDevice>(
            std::move(stream), [file]() { return io::read(file); }
        );
        std::string name;
        do {
            name = std::string("M.") + generate_random_base64<6>();
        } while (std::find(mounted.begin(), mounted.end(), name) != mounted.end());

        io::set_device(name, std::move(device));
        mounted.push_back(name);
        return name;
    }
    throw std::runtime_error("unable to mount " + file.string());
}

void EnginePaths::unmount(const std::string& name) {
    const auto& found = std::find(mounted.begin(), mounted.end(), name);
    if (found == mounted.end()) {
        throw std::runtime_error(name + " is not mounted");
    }
    io::remove_device(name);
    mounted.erase(found);
}

std::string EnginePaths::createMemoryDevice() {
    auto device = std::make_unique<io::MemoryDevice>();
        std::string name;
    do {
        name = std::string("W.") + generate_random_base64<6>();
    } while (std::find(mounted.begin(), mounted.end(), name) != mounted.end());
    
    io::set_device(name, std::move(device));
    mounted.push_back(name);
    return name;
}

std::string EnginePaths::createWriteableDevice(const std::string& name) {
    const auto& found = writeables.find(name);
    if (found != writeables.end()) {
        return found->second;
    }
    io::path folder;
    for (const auto& point : entryPoints) {
        if (point.name == name) {
            folder = point.path;
            break;
        }
    }
    if (name == "core") {
        folder = "res:";
    }
    if (folder.emptyOrInvalid()) {
        throw std::runtime_error("pack not found");
    }
    auto entryPoint = std::string("W.") + generate_random_base64<6>();
    io::create_subdevice(entryPoint, folder.entryPoint(), folder.pathPart());
    writeables[name] = entryPoint;
    return entryPoint;
}

void EnginePaths::cleanup() {
    // Remove previous content entry-points
    for (const auto& [id, _] : entryPoints) {
        io::remove_device(id);
    }
    for (const auto& [_, entryPoint] : writeables) {
        io::remove_device(entryPoint);
    }
    for (const auto& entryPoint : mounted) {
        io::remove_device(entryPoint);
    }
    entryPoints.clear();
    writeables.clear();
}

void EnginePaths::setEntryPoints(std::vector<PathsRoot> entryPoints) {
    cleanup();

    // Create sub-devices
    for (const auto& point : entryPoints) {
        auto parent = point.path.entryPoint();
        io::create_subdevice(point.name, parent, point.path);
    }
    this->entryPoints = std::move(entryPoints);
}

std::tuple<std::string, std::string> EnginePaths::parsePath(std::string_view path) {
    size_t separator = path.find(':');
    if (separator == std::string::npos) {
        return {"", std::string(path)};
    }
    auto prefix = std::string(path.substr(0, separator));
    auto filename = std::string(path.substr(separator + 1));
    return {prefix, filename};
}

ResPaths::ResPaths(std::vector<PathsRoot> roots)
    : roots(std::move(roots)) {
}

io::path ResPaths::find(const std::string& filename) const {
    for (int i = roots.size() - 1; i >= 0; i--) {
        auto& root = roots[i];
        auto file = root.path / filename;
        if (io::exists(file)) {
            return file;
        }
    }
    return io::path("res:") / filename;
}

std::string ResPaths::findRaw(const std::string& filename) const {
    for (int i = roots.size() - 1; i >= 0; i--) {
        auto& root = roots[i];
        if (io::exists(root.path / filename)) {
            return root.name + ":" + filename;
        }
    }
    throw std::runtime_error("could not to find file " + util::quote(filename));
}

std::vector<std::string> ResPaths::listdirRaw(const std::string& folderName) const {
    std::vector<std::string> entries;
    for (int i = roots.size() - 1; i >= 0; i--) {
        auto& root = roots[i];
        auto folder = root.path / folderName;
        if (!io::is_directory(folder)) continue;
        for (const auto& file : io::directory_iterator(folder)) {
            entries.emplace_back(root.name + ":" + folderName + "/" + file.name());
        }
    }
    return entries;
}

std::vector<io::path> ResPaths::listdir(
    const std::string& folderName
) const {
    std::vector<io::path> entries;
    for (int i = roots.size() - 1; i >= 0; i--) {
        auto& root = roots[i];
        io::path folder = root.path / folderName;
        if (!io::is_directory(folder)) continue;
        for (const auto& entry : io::directory_iterator(folder)) {
            entries.push_back(entry);
        }
    }
    return entries;
}

dv::value ResPaths::readCombinedList(const std::string& filename) const {
    dv::value list = dv::list();
    for (const auto& root : roots) {
        auto path = root.path / filename;
        if (!io::exists(path)) {
            continue;
        }
        try {
            auto value = io::read_object(path);
            if (!value.isList()) {
                logger.warning() << "reading combined list " << root.name << ":"
                    << filename << " is not a list (skipped)";
                continue;
            }
            for (const auto& elem : value) {
                list.add(elem);
            }
        } catch (const std::runtime_error& err) {
            logger.warning() << "reading combined list " << root.name << ":" 
                << filename << ": " << err.what();
        }
    }
    return list;
}

dv::value ResPaths::readCombinedObject(const std::string& filename, bool deep) const {
    dv::value object = dv::object();
    for (const auto& root : roots) {
        auto path = root.path / filename;
        if (!io::exists(path)) {
            continue;
        }
        try {
            auto value = io::read_object(path);
            if (!value.isObject()) {
                logger.warning()
                    << "reading combined object " << root.name << ": "
                    << filename << " is not an object (skipped)";
            }
            object.merge(std::move(value), deep);
        } catch (const std::runtime_error& err) {
            logger.warning() << "reading combined object " << root.name << ":"
                             << filename << ": " << err.what();
        }
    }
    return object;
}

std::vector<io::path> ResPaths::collectRoots() {
    std::vector<io::path> collected;
    collected.reserve(roots.size());
    for (const auto& root : roots) {
        collected.emplace_back(root.path);
    }
    return collected;
}
