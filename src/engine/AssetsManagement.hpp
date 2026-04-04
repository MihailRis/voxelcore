#pragma once

#include "assets/Assets.hpp"
#include "util/ObjectsKeeper.hpp"

#include <memory>

class AssetsLoader;
class Engine;
class Content;
class Task;
struct EngineSettings;

class AssetsManagement final {
public:
    AssetsManagement(Engine& engine);
    ~AssetsManagement();

    void loadAssets(Content* content);
    void update();
    
    Assets* getStorage();
    const Assets* getStorage() const;

    AssetsLoader& acquireBackgroundLoader();
private:
    void finishBackgroundLoader();

    Engine& engine;
    const EngineSettings& settings;
    std::unique_ptr<Assets> assets;
    std::unique_ptr<AssetsLoader> backgroundLoader;
    std::shared_ptr<Task> backgroundLoaderTask;
    util::ObjectsKeeper assetsVault;
};
