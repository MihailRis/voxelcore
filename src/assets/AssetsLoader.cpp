#include "AssetsLoader.hpp"

#include <iostream>
#include <memory>
#include <utility>

#include "coders/imageio.hpp"
#include "constants.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "debug/Logger.hpp"
#include "io/engine_paths.hpp"
#include "io/io.hpp"
#include "graphics/core/Texture.hpp"
#include "logic/scripting/scripting.hpp"
#include "objects/rigging.hpp"
#include "util/ThreadPool.hpp"
#include "voxels/Block.hpp"
#include "items/ItemDef.hpp"
#include "Assets.hpp"
#include "assetload_funcs.hpp"
#include "engine/Engine.hpp"

namespace fs = std::filesystem;

static debug::Logger logger("assets-loader");

AssetsLoader::AssetsLoader(Engine& engine, Assets& assets, const ResPaths& paths)
    : engine(engine), assets(assets), paths(paths) {
    addLoader(AssetType::SHADER, assetload::shader);
    addLoader(AssetType::TEXTURE, assetload::texture);
    addLoader(AssetType::FONT, assetload::font);
    addLoader(AssetType::ATLAS, assetload::atlas);
    addLoader(AssetType::LAYOUT, assetload::layout);
    addLoader(AssetType::SOUND, assetload::sound);
    addLoader(AssetType::MODEL, assetload::model);
    addLoader(AssetType::POST_EFFECT, assetload::posteffect);
}

void AssetsLoader::addLoader(AssetType tag, aloader_func func) {
    loaders[tag] = std::move(func);
}

void AssetsLoader::add(
    AssetType tag,
    const std::string& filename,
    const std::string& alias,
    std::shared_ptr<AssetCfg> settings
) {
    if (enqueued.find({tag, alias}) != enqueued.end()){
        return;
    }
    entries.push(aloader_entry {tag, filename, alias, std::move(settings)});
    enqueued.insert({tag, alias});
}

bool AssetsLoader::hasNext() const {
    return !entries.empty();
}

aloader_func AssetsLoader::getLoader(AssetType tag) {
    auto found = loaders.find(tag);
    if (found == loaders.end()) {
        throw std::runtime_error(
            "unknown asset tag " + std::to_string(static_cast<int>(tag))
        );
    }
    return found->second;
}

void AssetsLoader::loadNext() {
    const aloader_entry& entry = entries.front();
    logger.info() << "loading " << entry.filename << " as " << entry.alias;
    try {
        aloader_func loader = getLoader(entry.tag);
        auto postfunc =
            loader(this, paths, entry.filename, entry.alias, entry.config);
        postfunc(&assets);
        entries.pop();
    } catch (std::runtime_error& err) {
        logger.error() << err.what();
        auto type = entry.tag;
        std::string filename = entry.filename;
        std::string reason = err.what();
        entries.pop();
        throw assetload::error(type, std::move(filename), std::move(reason));
    }
}

static void add_layouts(
    const scriptenv& env,
    const std::string& prefix,
    const io::path& folder,
    AssetsLoader& loader
) {
    if (!io::is_directory(folder)) {
        return;
    }
    for (const auto& file : io::directory_iterator(folder)) {
        if (file.extension() != ".xml") continue;
        std::string name = prefix + ":" + file.stem();
        loader.add(
            AssetType::LAYOUT,
            file.string(),
            name,
            std::make_shared<LayoutCfg>(&loader.getEngine().getGUI(), env)
        );
    }
}

void AssetsLoader::tryAddSound(const std::string& name) {
    if (name.empty()) {
        return;
    }
    std::string file = SOUNDS_FOLDER + "/" + name;
    add(AssetType::SOUND, file, name);
}

static std::string assets_def_folder(AssetType tag) {
    switch (tag) {
        case AssetType::FONT:
            return FONTS_FOLDER;
        case AssetType::SHADER:
            return SHADERS_FOLDER;
        case AssetType::TEXTURE:
            return TEXTURES_FOLDER;
        case AssetType::ATLAS:
            return TEXTURES_FOLDER;
        case AssetType::LAYOUT:
            return LAYOUTS_FOLDER;
        case AssetType::SOUND:
            return SOUNDS_FOLDER;
        case AssetType::MODEL:
            return MODELS_FOLDER;
        case AssetType::POST_EFFECT:
            return POST_EFFECTS_FOLDER;
    }
    return "<error>";
}

void AssetsLoader::processPreload(
    AssetType tag, const std::string& name, const dv::value& map
) {
    std::string defFolder = assets_def_folder(tag);
    std::string path = defFolder + "/" + name;
    if (map == nullptr) {
        add(tag, path, name);
        return;
    }
    std::shared_ptr<AssetCfg> config = nullptr;
    map.at("path").get(path);
    switch (tag) {
        case AssetType::SOUND: {
            bool keepPCM = false;
            config = std::make_shared<SoundCfg>(map.at("keep-pcm").get(keepPCM));
            break;
        }
        case AssetType::ATLAS: {
            std::string typeName = "atlas";
            map.at("type").get(typeName);
            auto type = AtlasType::ATLAS;
            if (typeName == "separate") {
                type = AtlasType::SEPARATE;
            }
            config = std::make_shared<AtlasCfg>(type);
            break;
        }
        case AssetType::POST_EFFECT: {
            bool advanced = false;
            map.at("advanced").get(advanced);
            config = std::make_shared<PostEffectCfg>(advanced);
            break;
        }
        default:
            break;
    }
    add(tag, path, name, std::move(config));
}

void AssetsLoader::processPreloadList(AssetType tag, const dv::value& list) {
    if (list == nullptr) {
        return;
    }
    for (const auto& value : list) {
        switch (value.getType()) {
            case dv::value_type::string:
                processPreload(tag, value.asString(), nullptr);
                break;
            case dv::value_type::object:
                processPreload(tag, value["name"].asString(), value);
                break;
            default:
                throw std::runtime_error("invalid entry type");
        }
    }
}

void AssetsLoader::processPreloadConfig(const io::path& file) {
    auto root = io::read_json(file);
    processPreloadList(AssetType::ATLAS, root["atlases"]);
    processPreloadList(AssetType::FONT, root["fonts"]);
    processPreloadList(AssetType::SHADER, root["shaders"]);
    processPreloadList(AssetType::TEXTURE, root["textures"]);
    processPreloadList(AssetType::SOUND, root["sounds"]);
    processPreloadList(AssetType::MODEL, root["models"]);
    processPreloadList(AssetType::POST_EFFECT, root["post-effects"]);
    // layouts are loaded automatically
}

void AssetsLoader::processPreloadConfigs(const Content* content) {
    io::path preloadFile = "res:preload.json";
    if (io::exists(preloadFile)) {
        processPreloadConfig(preloadFile);
    }
    if (content == nullptr) {
        return;
    }
    for (auto& entry : content->getPacks()) {
        if (entry.first == "core") {
            continue;
        }
        const auto& pack = entry.second;
        preloadFile = pack->getInfo().folder / "preload.json";
        if (io::exists(preloadFile)) {
            processPreloadConfig(preloadFile);
        }
    }
}

static void add_variant(AssetsLoader& loader, const Variant& variant) {
    if (!variant.model.name.empty() &&
        variant.model.name.find(':') == std::string::npos) {
        loader.add(
            AssetType::MODEL,
            MODELS_FOLDER + "/" + variant.model.name,
            variant.model.name
        );
    }
}

void AssetsLoader::addDefaults(AssetsLoader& loader, const Content* content) {
    loader.processPreloadConfigs(content);
    if (content) {
        for (auto& entry : content->getBlockMaterials()) {
            auto& material = *entry.second;
            loader.tryAddSound(material.stepsSound);
            loader.tryAddSound(material.placeSound);
            loader.tryAddSound(material.breakSound);
            loader.tryAddSound(material.hitSound);
        }

        for (auto& entry : content->getPacks()) {
            auto pack = entry.second.get();
            auto& info = pack->getInfo();
            io::path folder = info.folder / "layouts";
            add_layouts(pack->getEnvironment(), info.id, folder, loader);
        }

        for (auto& entry : content->getSkeletons()) {
            auto& skeleton = *entry.second;
            for (auto& bone : skeleton.getBones()) {
                std::string model = bone->model.name;
                size_t pos = model.rfind('.');
                if (pos != std::string::npos) {
                    model = model.substr(0, pos);
                }
                if (!model.empty()) {
                    loader.add(
                        AssetType::MODEL, MODELS_FOLDER + "/" + model, model
                    );
                }
            }
        }
        for (const auto& [_, def] : content->blocks.getDefs()) {
            if (def->variants) {
                for (const auto& variant : def->variants->variants) {
                    add_variant(loader, variant);
                }
            } else {
                add_variant(loader, def->defaults);
            }
        }
        for (const auto& [_, def] : content->items.getDefs()) {
            if (def->modelName.find(':') == std::string::npos) {
                loader.add(
                    AssetType::MODEL,
                    MODELS_FOLDER + "/" + def->modelName,
                    def->modelName
                );
            }
        }
    }
}

bool AssetsLoader::loadExternalTexture(
    Assets* assets,
    const std::string& name,
    const std::vector<io::path>& alternatives
) {
    if (assets->get<Texture>(name) != nullptr) {
        return true;
    }
    for (auto& path : alternatives) {
        if (io::exists(path)) {
            try {
                auto image = imageio::read(path);
                assets->store(Texture::from(image.get()), name);
                return true;
            } catch (const std::exception& err) {
                logger.error() << "error while loading external "
                               << path.string() << ": " << err.what();
            }
        }
    }
    return false;
}

Engine& AssetsLoader::getEngine() {
    return engine;
}

const ResPaths& AssetsLoader::getPaths() const {
    return paths;
}

class LoaderWorker : public util::Worker<aloader_entry, assetload::postfunc> {
    AssetsLoader* loader;
public:
    LoaderWorker(AssetsLoader* loader) : loader(loader) {
    }

    assetload::postfunc operator()(const aloader_entry& entry
    ) override {
        aloader_func loadfunc = loader->getLoader(entry.tag);
        return loadfunc(
            loader,
            loader->getPaths(),
            entry.filename,
            entry.alias,
            entry.config
        );
    }
};

std::shared_ptr<Task> AssetsLoader::startTask(runnable onDone) {
    auto pool =
        std::make_shared<util::ThreadPool<aloader_entry, assetload::postfunc>>(
            "assets-loader-pool",
            [=]() { return std::make_shared<LoaderWorker>(this); },
            [this](const assetload::postfunc& func) { func(&assets); }
        );
    pool->setOnComplete(std::move(onDone));
    while (!entries.empty()) {
        aloader_entry entry = std::move(entries.front());
        entries.pop();
        pool->enqueueJob(std::move(entry));
    }
    return pool;
}
