#include "AssetsLoader.hpp"

#include "animation/rigging.hpp"
#include "assetload_funcs.hpp"
#include "Assets.hpp"
#include "coders/commons.hpp"
#include "coders/imageio.hpp"
#include "constants.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "graphics/core/Texture.hpp"
#include "io/io.hpp"
#include "items/ItemDef.hpp"
#include "logic/scripting/scripting.hpp"
#include "objects/EntityDef.hpp"
#include "util/ThreadPool.hpp"
#include "voxels/Block.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

static debug::Logger logger("assets-loader");

AssetsLoader::AssetsLoader(
    Engine& engine, Assets& assets, const ResPaths& paths
)
    : engine(engine),
      assets(assets),
      paths(paths),
      assetsLoadInfo(assets.getLoadInfo()) {
    addLoader(AssetType::ANIMATION, assetload::animation);
    addLoader(AssetType::ATLAS, assetload::atlas);
    addLoader(AssetType::FONT, assetload::font);
    addLoader(AssetType::LAYOUT, assetload::layout);
    addLoader(AssetType::MODEL, assetload::model);
    addLoader(AssetType::POST_EFFECT, assetload::posteffect);
    addLoader(AssetType::SHADER, assetload::shader);
    addLoader(AssetType::SKELETON, assetload::skeleton);
    addLoader(AssetType::SOUND, assetload::sound);
    addLoader(AssetType::TEXTURE, assetload::texture);
}

void AssetsLoader::addLoader(AssetType tag, aloader_func func) {
    loaders[tag] = std::move(func);
}

void AssetsLoader::add(
    AssetType tag,
    const std::string& filename,
    const std::string& alias,
    std::shared_ptr<AssetCfg> settings,
    bool overwrite
) {
    if (!overwrite && enqueued.find({alias, tag}) != enqueued.end()){
        return;
    }
    entries.push(aloader_entry {tag, filename, alias, std::move(settings)});
    enqueued.insert({alias, tag});
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
    aloader_entry& entry = entries.front();
    logger.info() << "loading " << entry.filename << " as " << entry.alias;

    std::string error {};
    try {
        aloader_func loader = getLoader(entry.tag);
        auto postfunc =
            loader(*this, paths, entry.filename, entry.alias, entry.config);
        postfunc(assets);
    } catch (const parsing_error& err) {
        error = err.errorLog();
    } catch (const std::runtime_error& err) {
        error = err.what();
    }
    if (!error.empty()) {
        logger.error() << error;
        auto tag = entry.tag;
        auto filename = entry.filename;
        entries.pop();
        throw assetload::error(tag, std::move(filename), std::move(error));
    }
    assetsLoadInfo.processedEntries[{entry.alias, entry.tag}] =
        std::move(entry);
    entries.pop();
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

static std::string assets_def_folder(AssetType tag) {
    switch (tag) {
        case AssetType::ANIMATION:
            return ANIMATION_FOLDER;
        case AssetType::ATLAS:
            return TEXTURES_FOLDER;
        case AssetType::FONT:
            return FONTS_FOLDER;
        case AssetType::LAYOUT:
            return LAYOUTS_FOLDER;
        case AssetType::MODEL:
            return MODELS_FOLDER;
        case AssetType::POST_EFFECT:
            return POST_EFFECTS_FOLDER;
        case AssetType::SHADER:
            return SHADERS_FOLDER;
        case AssetType::SKELETON:
            return SKELETONS_FOLDER;
        case AssetType::SOUND:
            return SOUNDS_FOLDER;
        case AssetType::TEXTURE:
            return TEXTURES_FOLDER;
    }
    return "<error>";
}

static std::shared_ptr<AssetCfg> create_preload_config(
    AssetType tag, const dv::value& map
) {
    switch (tag) {
        case AssetType::SOUND: {
            bool keepPCM = false;
            map.at("keep-pcm").get(keepPCM);
            return std::make_shared<SoundCfg>(keepPCM);
        }
        case AssetType::ATLAS: {
            std::string typeName = "atlas";
            map.at("type").get(typeName);
            auto type = AtlasType::ATLAS;
            if (typeName == "separate") {
                type = AtlasType::SEPARATE;
            }
            return std::make_shared<AtlasCfg>(type);
        }
        case AssetType::POST_EFFECT: {
            bool advanced = false;
            map.at("advanced").get(advanced);
            return std::make_shared<PostEffectCfg>(advanced);
        }
        case AssetType::MODEL: {
            bool squashed = false;
            map.at("squash").get(squashed);
            return std::make_shared<ModelCfg>(squashed);
        }
        case AssetType::FONT: {
            int size = DEFAULT_FONT_SIZE;
            map.at("size").get(size);
            return std::make_shared<FontCfg>(size);
        }
        default:
            return nullptr;
    }
}

static bool is_pattern(const std::string& name) {
    return name.find('*') != std::string::npos;
}

static bool match_wildcard(std::string_view pattern, std::string_view str) {
    size_t p = 0;
    size_t s = 0;
    size_t starPos = std::string_view::npos;
    size_t starMatch = 0;
    while (s < str.length()) {
        if (p < pattern.length() && pattern[p] == '*') {
            starPos = p++;
            starMatch = s;
        } else if (p < pattern.length() && pattern[p] == str[s]) {
            p++;
            s++;
        } else if (starPos != std::string_view::npos) {
            p = starPos + 1;
            s = ++starMatch;
        } else {
            return false;
        }
    }
    while (p < pattern.length() && pattern[p] == '*') {
        p++;
    }
    return p == pattern.length();
}

static std::vector<std::string> split_pattern(const std::string& pattern) {
    if (pattern.find("**") != std::string::npos) {
        throw std::runtime_error(
            "'**' is not supported in preload pattern " + util::quote(pattern)
        );
    }
    std::vector<std::string> segments;
    size_t start = 0;
    while (start < pattern.length()) {
        size_t end = pattern.find('/', start);
        if (end == std::string::npos) {
            end = pattern.length();
        }
        if (end > start) {
            auto segment = pattern.substr(start, end - start);
            if (segment == "." || segment == "..") {
                throw std::runtime_error(
                    "relative segments are not allowed in preload pattern " +
                    util::quote(pattern)
                );
            }
            segments.push_back(std::move(segment));
        }
        start = end + 1;
    }
    return segments;
}

static void glob(
    const io::path& folder,
    const std::vector<std::string>& segments,
    size_t index,
    const std::string& relative,
    bool directories,
    std::vector<std::string>& out
) {
    const auto& segment = segments[index];
    bool last = index + 1 == segments.size();

    auto visit = [&](const std::string& name) {
        io::path path = folder / name;
        std::string rel = relative.empty() ? name : relative + "/" + name;
        if (!last) {
            if (io::is_directory(path)) {
                glob(path, segments, index + 1, rel, directories, out);
            }
        } else if (directories ? io::is_directory(path)
                               : io::is_regular_file(path)) {
            out.push_back(std::move(rel));
        }
    };

    if (!is_pattern(segment)) {
        visit(segment);
        return;
    }
    if (!io::is_directory(folder)) {
        return;
    }
    for (const auto& entry : io::directory_iterator(folder)) {
        auto name = entry.name();
        if (match_wildcard(segment, name)) {
            visit(name);
        }
    }
}

static std::vector<std::string> asset_search_folders(AssetType tag) {
    if (tag == AssetType::ANIMATION) {
        return {ANIMATION_FOLDER, ANIMATION_FOLDER_ALIAS};
    }
    return {assets_def_folder(tag)};
}

static std::vector<std::string> asset_extensions(AssetType tag) {
    switch (tag) {
        case AssetType::ANIMATION:
            return {".vca"};
        case AssetType::FONT:
            return {".png", ".ttf", ".otf"};
        case AssetType::MODEL:
            return {".vec3", ".obj", ".xml", ".vcm"};
        case AssetType::POST_EFFECT:
            return {".glsl"};
        case AssetType::SHADER:
            return {".glslv", ".glslf"};
        case AssetType::SKELETON:
            return {".json"};
        case AssetType::SOUND:
            return {".ogg", ".wav"};
        case AssetType::TEXTURE:
            return {".png"};
        default:
            return {};
    }
}

static std::optional<std::string> strip_index_suffix(const std::string& stem) {
    size_t pos = stem.rfind('_');
    if (pos == std::string::npos || pos + 1 == stem.length()) {
        return std::nullopt;
    }
    for (size_t i = pos + 1; i < stem.length(); i++) {
        if (!std::isdigit(static_cast<unsigned char>(stem[i]))) {
            return std::nullopt;
        }
    }
    return stem.substr(0, pos);
}

struct PreloadMatch {
    std::string name;
    std::string path;
};

/// @brief Resolve asset name for a file matched by preload pattern
/// @param folder asset type folder the file was found in
/// @param relative file path relative to the folder
/// @return std::nullopt if the file is not a loadable asset of the type
static std::optional<PreloadMatch> resolve_preload_file(
    AssetType tag, const io::path& folder, const std::string& relative
) {
    io::path file = relative;
    std::string extension = file.extension();
    const auto& extensions = asset_extensions(tag);
    if (std::find(extensions.begin(), extensions.end(), extension) ==
        extensions.end()) {
        return std::nullopt;
    }
    std::string base = relative.substr(0, relative.length() - extension.length());
    std::string defFolder = assets_def_folder(tag);

    if (tag == AssetType::FONT) {
        if (extension != ".png") {
            return PreloadMatch {base, defFolder + "/" + relative};
        }
        auto fontName = strip_index_suffix(base);
        if (!fontName || !io::exists(folder / (*fontName + "_0.png"))) {
            return std::nullopt;
        }
        base = std::move(*fontName);
    } else if (tag == AssetType::SOUND) {
        auto soundName = strip_index_suffix(base);
        if (soundName && (io::exists(folder / (*soundName + extension)) ||
                          io::exists(folder / (*soundName + "_0" + extension)))) {
            base = std::move(*soundName);
        }
    }
    return PreloadMatch {base, defFolder + "/" + base};
}

static std::vector<PreloadMatch> find_preload_matches(
    AssetType tag, const io::path& packFolder, const std::string& pattern
) {
    auto segments = split_pattern(pattern);
    if (segments.empty()) {
        return {};
    }
    bool directories = pattern.back() == '/' || tag == AssetType::ATLAS;
    if (directories && tag != AssetType::ATLAS) {
        throw std::runtime_error(
            "directory pattern " + util::quote(pattern) +
            " is not applicable to assets of type " + assets_def_folder(tag)
        );
    }
    std::map<std::string, std::string> found;
    for (const auto& searchFolder : asset_search_folders(tag)) {
        io::path folder = packFolder / searchFolder;
        std::vector<std::string> files;
        glob(folder, segments, 0, "", directories, files);
        for (const auto& relative : files) {
            if (directories) {
                found.try_emplace(relative, assets_def_folder(tag) + "/" + relative);
                continue;
            }
            if (auto match = resolve_preload_file(tag, folder, relative)) {
                found.try_emplace(std::move(match->name), std::move(match->path));
            }
        }
    }
    std::vector<PreloadMatch> matches;
    matches.reserve(found.size());
    for (auto& [name, path] : found) {
        matches.push_back(PreloadMatch {name, std::move(path)});
    }
    return matches;
}

void AssetsLoader::processPreload(
    AssetType tag,
    const std::string& name,
    const dv::value& map,
    const io::path& packFolder
) {
    std::shared_ptr<AssetCfg> config =
        map == nullptr ? nullptr : create_preload_config(tag, map);

    if (is_pattern(name)) {
        if (map != nullptr && map.has("path")) {
            throw std::runtime_error(
                "'path' cannot be used with preload pattern " + util::quote(name)
            );
        }
        auto matches = find_preload_matches(tag, packFolder, name);
        if (matches.empty()) {
            logger.warning() << "preload pattern " << util::quote(name)
                             << " matches nothing in " << packFolder.string();
        }
        for (const auto& match : matches) {
            logger.debug() << "processing preload " << util::quote(match.name)
                           << " path: " << util::quote(match.path);
            add(tag, match.path, match.name, config, true);
        }
        return;
    }

    std::string path = assets_def_folder(tag) + "/" + name;
    if (map != nullptr) {
        map.at("path").get(path);
        logger.debug() << "processing preload " << util::quote(name)
                       << " path: " << util::quote(path);
    }
    add(tag, path, name, std::move(config), true);
}

void AssetsLoader::processPreloadList(
    AssetType tag, const dv::value& list, const io::path& packFolder
) {
    if (list == nullptr) {
        return;
    }
    for (const auto& value : list) {
        switch (value.getType()) {
            case dv::value_type::string:
                processPreload(tag, value.asString(), nullptr, packFolder);
                break;
            case dv::value_type::object:
                processPreload(tag, value["name"].asString(), value, packFolder);
                break;
            default:
                throw std::runtime_error("invalid entry type");
        }
    }
}

void AssetsLoader::processPreloadConfig(const io::path& file) {
    auto root = io::read_json(file);
    io::path folder = file.parent();
    processPreloadList(AssetType::ANIMATION, root["animations"], folder);
    processPreloadList(AssetType::ANIMATION, root["animation"], folder);
    processPreloadList(AssetType::ATLAS, root["atlases"], folder);
    processPreloadList(AssetType::FONT, root["fonts"], folder);
    processPreloadList(AssetType::MODEL, root["models"], folder);
    processPreloadList(AssetType::POST_EFFECT, root["post-effects"], folder);
    processPreloadList(AssetType::SHADER, root["shaders"], folder);
    processPreloadList(AssetType::SKELETON, root["skeletons"], folder);
    processPreloadList(AssetType::SOUND, root["sounds"], folder);
    processPreloadList(AssetType::TEXTURE, root["textures"], folder);
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

void AssetsLoader::addDefaults(const Content* content) {
    processPreloadConfigs(content);
    if (content == nullptr) {
        return;
    }
    auto tryAddSound = [this](const std::string& name){
        if (name.empty()) {
            return;
        }
        std::string file = SOUNDS_FOLDER + "/" + name;
        add(AssetType::SOUND, file, name);
    };
    for (auto& entry : content->getBlockMaterials()) {
        auto& material = *entry.second;
        tryAddSound(material.stepsSound);
        tryAddSound(material.placeSound);
        tryAddSound(material.breakSound);
        tryAddSound(material.hitSound);
    }

    for (auto& entry : content->getPacks()) {
        auto pack = entry.second.get();
        auto& info = pack->getInfo();
        io::path folder = info.folder / "layouts";
        add_layouts(pack->getEnvironment(), info.id, folder, *this);
    }

    for (const auto& entry : content->getPacks()) {
        io::path skeletonsDir = entry.first + ":skeletons";
        if (!io::is_directory(skeletonsDir)) {
            continue;
        }
        for (const auto& file : io::directory_iterator(skeletonsDir)) {
            add(
                AssetType::SKELETON,
                (file.parent() / file.stem()).string(),
                entry.first + ":" + file.stem()
            );
        }
    }

    for (const auto& [_, def] : content->blocks.getDefs()) {
        if (def->variants) {
            for (const auto& variant : def->variants->variants) {
                add_variant(*this, variant);
            }
        } else {
            add_variant(*this, def->defaults);
        }
    }
    for (const auto& [_, def] : content->items.getDefs()) {
        if (def->modelName.find(':') == std::string::npos) {
            add(
                AssetType::MODEL,
                MODELS_FOLDER + "/" + def->modelName,
                def->modelName
            );
        }
    }
    for (const auto& [_, def] : content->entities.getDefs()) {
        if (def->skeletonName.find(':') == std::string::npos) {
            // expecting a VCM with skeleton
            add(
                AssetType::MODEL,
                MODELS_FOLDER + "/" + def->skeletonName,
                def->skeletonName
            );
        }
    }
}

bool AssetsLoader::loadExternalTexture(
    AssetsLoader& loader,
    const std::string& name,
    const std::vector<io::path>& alternatives
) {
    if (loader.getAssets().get<Texture>(name) != nullptr) {
        return true;
    }
    for (auto& path : alternatives) {
        if (io::exists(path)) {
            loader.add(
                AssetType::TEXTURE,
                (path.parent() / path.stem()).string(),
                name,
                nullptr
            );
            return true;
        }
    }
    return false;
}

Assets& AssetsLoader::getAssets() {
    return assets;
}

Engine& AssetsLoader::getEngine() {
    return engine;
}

const ResPaths& AssetsLoader::getPaths() const {
    return paths;
}

class LoaderWorker : public util::Worker<aloader_entry, assetload::postfunc> {
    AssetsLoader& loader;
public:
    LoaderWorker(AssetsLoader& loader) : loader(loader) {
    }

    assetload::postfunc operator()(const aloader_entry& entry
    ) override {
        aloader_func loadfunc = loader.getLoader(entry.tag);
        return loadfunc(
            loader,
            loader.getPaths(),
            entry.filename,
            entry.alias,
            entry.config
        );
    }
};

std::shared_ptr<Task> AssetsLoader::startTask(runnable onDone, int maxWorkers) {
    auto pool =
        std::make_shared<util::ThreadPool<aloader_entry, assetload::postfunc>>(
            "assets-loader-pool",
            [=]() { return std::make_unique<LoaderWorker>(*this); },
            [this](const assetload::postfunc& func) { func(assets); },
            maxWorkers
        );
    pool->setOnComplete(std::move(onDone));
    pool->setJobsSource([this]() -> std::optional<aloader_entry> {
        if (entries.empty()) {
            return std::nullopt;
        }
        aloader_entry entry = std::move(entries.front());
        entries.pop();
        return entry;
    });

    while (!entries.empty()) {
        aloader_entry entry = std::move(entries.front());
        entries.pop();
        pool->enqueueJob(std::move(entry));
    }
    return pool;
}

void AssetsLoader::attachToFile(const io::path& file, AssetFullId assetId) {
    assetsLoadInfo.referencedAssets.insert({file, std::move(assetId)});
}

int AssetsLoader::addReload(const io::path& path) {
    int added = 0;

    auto range = assetsLoadInfo.referencedAssets.equal_range(path);
    for (auto it = range.first; it != range.second; ++it) {
        const auto& recipe = assetsLoadInfo.processedEntries.find(it->second);
        if (recipe != assetsLoadInfo.processedEntries.end()) {
            entries.push(recipe->second);
            added++;
        }
    }

    return added;
}
