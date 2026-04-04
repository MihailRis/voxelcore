#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>

class Engine;
class World;
class ContentReport;
class LevelController;

class EngineController {
    Engine& engine;

    int64_t localPlayer = -1;
    void onMissingContent(const std::shared_ptr<ContentReport>& report);
public:
    EngineController(Engine& engine);

    /// @brief Load world, convert if required and set to LevelScreen.
    /// @param name world name
    /// @param confirmConvert automatically confirm convert if requested
    void openWorld(const std::string& name, bool confirmConvert);

    /// @brief Show world removal confirmation dialog
    /// @param name world name
    void deleteWorld(const std::string& name);

    void reconfigPacks(
        LevelController* controller,
        const std::vector<std::string>& packsToAdd,
        const std::vector<std::string>& packsToRemove
    );

    void createWorld(
        const std::string& name,
        const std::string& seedstr,
        const std::string& generatorID
    );

    void setLocalPlayer(int64_t player);

    void reopenWorld(World* world);
};
