#pragma once

#include <memory>

class Level;
class Assets;
class Player;
class Engine;
class ContentGfxCache;
class LevelController;
struct EngineSettings;

class LevelFrontend {
    Level& level;
    LevelController* controller;
    Assets& assets;
    std::unique_ptr<ContentGfxCache> contentCache;
public:
    LevelFrontend(
        Engine& engine,
        Player* currentPlayer,
        LevelController* controller,
        const EngineSettings& settings
    );
    ~LevelFrontend();

    Level& getLevel();
    const ContentGfxCache& getContentGfxCache() const;
    ContentGfxCache& getContentGfxCache();
    LevelController* getController() const;
};
