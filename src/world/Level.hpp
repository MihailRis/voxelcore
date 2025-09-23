#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "typedefs.hpp"

class Content;
class World;
class Entities;
class Inventories;
class LevelEvents;
class PhysicsSolver;
class GlobalChunks;
class Camera;
class Players;
struct EngineSettings;

namespace voxels {
    class Pathfinding;
}

/// @brief A level, contains chunks and objects
class Level {
    std::unique_ptr<World> world;
public:
    const Content& content;
    
    std::unique_ptr<GlobalChunks> chunks;
    std::unique_ptr<Inventories> inventories;
    std::unique_ptr<PhysicsSolver> physics;
    std::unique_ptr<LevelEvents> events;
    std::unique_ptr<Entities> entities;
    std::unique_ptr<Players> players;
    std::unique_ptr<voxels::Pathfinding> pathfinding;
    std::vector<std::shared_ptr<Camera>> cameras;  // move somewhere?

    Level(
        std::unique_ptr<World> world,
        const Content& content,
        EngineSettings& settings
    );
    ~Level();

    World* getWorld();

    const World* getWorld() const;

    void onSave();

    std::shared_ptr<Camera> getCamera(const std::string& name);
};
