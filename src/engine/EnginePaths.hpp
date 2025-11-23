#pragma once

#include "io/io.hpp"
#include "data/dv.hpp"
#include "CoreParameters.hpp"

#include <unordered_map>
#include <optional>
#include <string>
#include <vector>
#include <tuple>

struct PathsRoot {
    std::string name;
    io::path path;

    PathsRoot(std::string name, io::path path)
        : name(std::move(name)), path(std::move(path)) {
    }
};

class ResPaths {
public:
    ResPaths() = default;

    ResPaths(std::vector<PathsRoot> roots);

    io::path find(const std::string& filename) const;
    std::string findRaw(const std::string& filename) const;
    std::vector<io::path> listdir(const std::string& folder) const;
    std::vector<std::string> listdirRaw(const std::string& folder) const;

    /// @brief Read all found list versions from all packs and combine into a
    /// single list. Invalid versions will be skipped with logging a warning
    /// @param file *.json file path relative to entry point 
    dv::value readCombinedList(const std::string& file) const;

    dv::value readCombinedObject(const std::string& file, bool deep=false) const;

    std::vector<io::path> collectRoots();
private:
    std::vector<PathsRoot> roots;
};

class EnginePaths {
public:
    ResPaths resPaths;

    EnginePaths(CoreParameters& params);

    std::filesystem::path getResourcesFolder() const;
    std::filesystem::path getUserFilesFolder() const;

    io::path getWorldFolderByName(const std::string& name);
    io::path getWorldsFolder() const;

    void setCurrentWorldFolder(io::path folder);
    io::path getNewScreenshotFile(const std::string& ext) const;

    std::string mount(const io::path& file);
    void unmount(const std::string& name);

    std::string createWriteableDevice(const std::string& name);
    std::string createMemoryDevice();

    void setEntryPoints(std::vector<PathsRoot> entryPoints);

    std::vector<io::path> scanForWorlds() const;

    static std::tuple<std::string, std::string> parsePath(std::string_view view);

    static inline io::path CONFIG_DEFAULTS = "config/defaults.toml";
    static inline io::path CONTROLS_FILE = "user:controls.toml";
    static inline io::path SETTINGS_FILE = "user:settings.toml";
private:
    std::filesystem::path resourcesFolder;
    std::filesystem::path userFilesFolder;
    std::filesystem::path projectFolder;
    io::path currentWorldFolder;
    std::optional<std::filesystem::path> scriptFolder;
    std::vector<PathsRoot> entryPoints;
    std::unordered_map<std::string, std::string> writeables;
    std::vector<std::string> mounted;

    void cleanup();
};
