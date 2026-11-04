#pragma once

#include <glm/glm.hpp>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#include "content/ContentPack.hpp"
#include "typedefs.hpp"
#include "voxels/Chunk.hpp"
#include "WorldRegions.hpp"
#include "io/io.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

inline constexpr uint WORLD_FORMAT_VERSION = 1;

class Player;
class Content;
class ContentIndices;
class World;
struct WorldInfo;
struct DebugSettings;

struct ContentRemovalRequest {
    std::vector<std::string> units;
};

class WorldFiles {
    io::path directory;
    WorldRegions regions;

    bool generatorTestMode = false;
    bool doWriteLights = true;

    io::path getWorldFile() const;
    io::path getPacksFile() const;

    void writeWorldInfo(const WorldInfo& info);
    void writeIndices(const ContentIndices* indices);
public:
    WorldFiles(const io::path& directory);
    WorldFiles(const io::path& directory, const DebugSettings& settings);
    ~WorldFiles();

    io::path getPlayerFile() const;
    io::path getIndicesFile() const;
    io::path getResourcesFile() const;
    void createDirectories();

    std::optional<WorldInfo> readWorldInfo();
    bool readResourcesData(const Content& content);

    static void createContentIndicesCache(
        const ContentIndices* indices, dv::value& root
    );
    static void createBlockFieldsIndices(
        const ContentIndices* indices, dv::value& root
    );

    void patchIndicesFile(const dv::value& map);

    /// @brief Write all unsaved data to world files
    /// @param world target world
    /// @param content world content
    void write(const World* world, const Content* content);

    void writePacks(const std::vector<ContentPack>& packs);

    void removeIndices(const std::vector<std::string>& packs);

    /// @return world folder
    io::path getFolder() const;

    WorldRegions& getRegions() {
        return regions;
    }

    bool doesWriteLights() const {
        return doWriteLights;
    }

    void deleteContent(const ContentRemovalRequest& request);

    static const inline std::string WORLD_FILE = "world.json";
};
