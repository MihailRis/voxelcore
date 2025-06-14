#pragma once

#include <functional>
#include <map>
#include <set>
#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "delegates.hpp"
#include "interfaces/Task.hpp"
#include "typedefs.hpp"
#include "Assets.hpp"
#include "data/dv.hpp"
#include "io/fwd.hpp"

class ResPaths;
class AssetsLoader;
class Content;
class Engine;

namespace gui {
    class GUI;
}

struct AssetCfg {
    virtual ~AssetCfg() {
    }
};

struct LayoutCfg : AssetCfg {
    gui::GUI* gui;
    scriptenv env;

    LayoutCfg(gui::GUI* gui, scriptenv env) : gui(gui), env(std::move(env)) {
    }
};

struct SoundCfg : AssetCfg {
    bool keepPCM;

    SoundCfg(bool keepPCM) : keepPCM(keepPCM) {}
};

enum class AtlasType {
    ATLAS, SEPARATE
};

struct AtlasCfg : AssetCfg {
    AtlasType type;

    AtlasCfg(AtlasType type) : type(type) {}
};

struct PostEffectCfg : AssetCfg {
    bool advanced;

    PostEffectCfg(bool advanced) : advanced(advanced) {}
};

using aloader_func = std::function<
    assetload::
        postfunc(AssetsLoader*, const ResPaths&, const std::string&, const std::string&, std::shared_ptr<AssetCfg>)>;

struct aloader_entry {
    AssetType tag;
    std::string filename;
    std::string alias;
    std::shared_ptr<AssetCfg> config;
};

class AssetsLoader {
    Engine& engine;
    Assets& assets;
    std::map<AssetType, aloader_func> loaders;
    std::queue<aloader_entry> entries;
    std::set<std::pair<AssetType, std::string>> enqueued;
    const ResPaths& paths;

    void tryAddSound(const std::string& name);

    void processPreload(
        AssetType tag, const std::string& name, const dv::value& map
    );
    void processPreloadList(AssetType tag, const dv::value& list);
    void processPreloadConfig(const io::path& file);
    void processPreloadConfigs(const Content* content);
public:
    AssetsLoader(Engine& engine, Assets& assets, const ResPaths& paths);
    void addLoader(AssetType tag, aloader_func func);

    /// @brief Enqueue asset load
    /// @param tag asset type
    /// @param filename asset file path
    /// @param alias internal asset name
    /// @param settings asset loading settings (based on asset type)
    void add(
        AssetType tag,
        const std::string& filename,
        const std::string& alias,
        std::shared_ptr<AssetCfg> settings = nullptr
    );

    bool hasNext() const;

    /// @throws assetload::error
    void loadNext();

    std::shared_ptr<Task> startTask(runnable onDone);

    const ResPaths& getPaths() const;
    aloader_func getLoader(AssetType tag);

    /// @brief Enqueue core and content assets
    /// @param loader target loader
    /// @param content engine content
    static void addDefaults(AssetsLoader& loader, const Content* content);

    static bool loadExternalTexture(
        Assets* assets,
        const std::string& name,
        const std::vector<io::path>& alternatives
    );

    Engine& getEngine();
};
