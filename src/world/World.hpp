#pragma once

#include <memory>
#include <string>
#include <vector>

#include "content/ContentPack.hpp"
#include "interfaces/Serializable.hpp"
#include "io/fwd.hpp"
#include "typedefs.hpp"
#include "util/timeutil.hpp"

class Content;
class WorldFiles;
class Level;
class ContentReport;
struct EngineSettings;

class world_load_error : public std::runtime_error {
public:
    world_load_error(const std::string& message);
};

struct WorldInfo : public Serializable {
    std::string name;
    std::string generator;
    uint64_t seed;
    int64_t nextInventoryId = 1;
    int64_t nextPlayerId = 0;

    /// @brief Day/night loop timer in range 0..1 where
    /// 0.0 - is midnight and
    /// 0.5 - is noon
    float daytime = timeutil::time_value(10, 00, 00);

    float daytimeSpeed = 1.0f;

    /// @brief total time passed in the world (not depending on daytimeSpeed)
    double totalTime = 0.0;

    /// @brief will be replaced with weather in future
    float fog = 0.0f;

    entityid_t nextEntityId = 0;

    int major = 0, minor = -1;

    bool isLoaded = false;

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};

/// @brief holds all world data except the level (chunks and objects)
class World {
    WorldInfo info {};

    const Content& content;
    std::vector<ContentPack> packs;

    void writeResources(const Content& content);
public:
    std::shared_ptr<WorldFiles> wfile;

    World(
        WorldInfo info,
        const std::shared_ptr<WorldFiles>& worldFiles,
        const Content& content,
        const std::vector<ContentPack>& packs
    );

    ~World();

    /// @brief Update world day-time and total time
    /// @param delta delta-time
    void updateTimers(float delta);

    /// @brief Write all unsaved level data to the world directory
    void write(Level* level);

    /// @brief Check world indices and generate ContentReport if convert required
    /// @param directory world directory
    /// @param content current Content instance
    /// @return ContentReport if world convert required else nullptr
    static std::shared_ptr<ContentReport> checkIndices(
        const std::shared_ptr<WorldFiles>& worldFiles, const Content* content
    );

    /// @brief Create new world
    /// @param name internal world name
    /// @param directory root world directory
    /// @param type of the world
    /// @param seed world generation seed
    /// @param settings current engine settings
    /// @param content current engine Content instance
    /// with all world content-packs applied
    /// @param packs vector of all world content-packs
    /// @return Level instance containing World instance
    static std::unique_ptr<Level> create(
        const std::string& name,
        const std::string& generator,
        const io::path& directory,
        uint64_t seed,
        EngineSettings& settings,
        const Content& content,
        const std::vector<ContentPack>& packs
    );

    /// @brief Load an existing world
    /// @param worldFiles world files manager
    /// @param settings current engine settings
    /// @param content current engine Content instance
    /// with all world content-packs applied
    /// @param packs vector of all world content-packs
    /// @return Level instance containing World instance
    /// @throws world_load_error on world.json load error
    static std::unique_ptr<Level> load(
        const std::shared_ptr<WorldFiles>& worldFiles,
        EngineSettings& settings,
        const Content& content,
        const std::vector<ContentPack>& packs
    );

    void setName(const std::string& name);
    void setSeed(uint64_t seed);
    void setGenerator(const std::string& generator);

    /// @brief Check if world has content-pack installed
    /// @param id content-pack id
    bool hasPack(const std::string& id) const;

    /// @brief Get internal world name (not the folder name)
    /// @return name stored in world.json
    std::string getName() const;

    /// @brief Get world generation seed
    uint64_t getSeed() const;

    /// @brief Get world generator id
    std::string getGenerator() const;

    bool isNameless() const {
        return info.name.empty();
    }

    WorldInfo& getInfo() {
        return info;
    }

    const WorldInfo& getInfo() const {
        return info;
    }

    /// @brief Get vector of all content-packs installed in world
    const std::vector<ContentPack>& getPacks() const;

    /// @brief Get next inventory id and increment it's counter
    /// @return integer >= 1
    int64_t getNextInventoryId() {
        return info.nextInventoryId++;
    }
};
