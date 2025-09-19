#define VC_ENABLE_REFLECTION
#include "ContentLoader.hpp"

#include <algorithm>
#include <glm/glm.hpp>
#include <iostream>

#include "loading/ContentUnitLoader.hpp"
#include "ContentBuilder.hpp"
#include "ContentPack.hpp"
#include "debug/Logger.hpp"
#include "logic/scripting/scripting.hpp"
#include "objects/rigging.hpp"
#include "util/listutil.hpp"
#include "util/stringutil.hpp"
#include "io/engine_paths.hpp"

static debug::Logger logger("content-loader");

ContentLoader::ContentLoader(
    ContentPack* pack, ContentBuilder& builder, const ResPaths& paths
)
    : pack(pack), builder(builder), paths(paths) {
    auto runtime = std::make_unique<ContentPackRuntime>(
        *pack, scripting::create_pack_environment(*pack)
    );
    stats = &runtime->getStatsWriteable();
    env = runtime->getEnvironment();
    this->runtime = runtime.get();
    builder.add(std::move(runtime));
}

static void detect_defs(
    const io::path& folder,
    const std::string& prefix,
    std::vector<std::string>& detected
) {
    if (!io::is_directory(folder)) {
        return;
    }
    for (const auto& file : io::directory_iterator(folder)) {
        std::string name = file.stem();
        if (name[0] == '_') {
            continue;
        }
        if (io::is_regular_file(file) && io::is_data_file(file)) {
            auto map = io::read_object(file);
            std::string id = prefix.empty() ? name : prefix + ":" + name;
            detected.emplace_back(id);
        } else if (io::is_directory(file) && file.extension() != ".files") {
            detect_defs(file, name, detected);
        }
    }
}

static void detect_defs_pairs(
    const io::path& folder,
    const std::string& prefix,
    std::vector<std::tuple<std::string, std::string>>& detected
) {
    if (!io::is_directory(folder)) {
        return;
    }
    for (const auto& file : io::directory_iterator(folder)) {
        std::string name = file.stem();
        if (name[0] == '_') {
            continue;
        }
        if (io::is_regular_file(file) && io::is_data_file(file)) {
            try {
                auto map = io::read_object(file);
                auto id = prefix.empty() ? name : prefix + ":" + name;
                auto caption = util::id_to_caption(id);
                map.at("caption").get(caption);
                detected.emplace_back(id, name);
            } catch (const std::runtime_error& err) {
                logger.error() << err.what();
            }
        } else if (io::is_directory(file) && file.extension() != ".files") {
            detect_defs_pairs(file, name, detected);
        }
    }
}

std::vector<std::tuple<std::string, std::string>> ContentLoader::scanContent(
    const ContentPack& pack, ContentType type
) {
    std::vector<std::tuple<std::string, std::string>> detected;
    detect_defs_pairs(
        pack.folder / ContentPack::getFolderFor(type), pack.id, detected);
    return detected;
}

bool ContentLoader::fixPackIndices(
    const io::path& folder,
    dv::value& indicesRoot,
    const std::string& contentSection
) {
    std::vector<std::string> detected;
    detect_defs(folder, "", detected);

    std::vector<std::string> indexed;
    bool modified = false;
    if (!indicesRoot.has(contentSection)) {
        indicesRoot.list(contentSection);
    }
    auto& arr = indicesRoot[contentSection];
    for (size_t i = 0; i < arr.size(); i++) {
        const std::string& name = arr[i].asString();
        if (!util::contains(detected, name)) {
            arr.erase(i);
            i--;
            modified = true;
            continue;
        }
        indexed.push_back(name);
    }
    for (auto name : detected) {
        if (!util::contains(indexed, name)) {
            arr.add(name);
            modified = true;
        }
    }
    return modified;
}

void ContentLoader::fixPackIndices() {
    auto folder = pack->folder;
    auto contentFile = pack->getContentFile();
    auto blocksFolder = folder / ContentPack::BLOCKS_FOLDER;
    auto itemsFolder = folder / ContentPack::ITEMS_FOLDER;
    auto entitiesFolder = folder / ContentPack::ENTITIES_FOLDER;

    dv::value root;
    if (io::is_regular_file(contentFile)) {
        root = io::read_json(contentFile);
    } else {
        root = dv::object();
    }

    bool modified = false;
    modified |= fixPackIndices(blocksFolder, root, "blocks");
    modified |= fixPackIndices(itemsFolder, root, "items");
    modified |= fixPackIndices(entitiesFolder, root, "entities");

    if (modified) {
        // rewrite modified json
        io::write_json(contentFile, root);
    }
}

void process_method(
    dv::value& properties,
    const std::string& method,
    const std::string& name,
    const dv::value& value
) {
    if (method == "append") {
        if (!properties.has(name)) {
            properties[name] = dv::list();
        }
        auto& list = properties[name];
        if (value.isList()) {
            for (const auto& item : value) {
                list.add(item);
            }
        } else {
            list.add(value);
        }
    } else {
        throw std::runtime_error(
            "unknown method " + method + " for " + name
        );
    }
}

template<typename DefT> 
void ContentUnitLoader<DefT>::loadUnit(
    DefT& def, const std::string& full, const std::string& name
) {
    auto folder = pack.folder;
    auto configFile = folder / (defsDir + "/" + name + ".json");
    if (io::exists(configFile)) loadUnit(def, full, configFile);
}

void ContentLoader::loadBlockMaterial(
    BlockMaterial& def, const io::path& file
) {
    def.deserialize(io::read_json(file));
    if (def.hitSound.empty()) {
        def.hitSound = def.stepsSound;
    }
}

template <typename DefT>
void ContentUnitLoader<DefT>::loadDefs(const dv::value& root) {
    auto found = root.at(defsDir);
    if (!found) {
        return;
    }
    const auto& defsArr = *found;

    std::vector<std::pair<std::string, std::string>> pendingDefs;
    auto getJsonParent = [this](const std::string& prefix, const std::string& name) {
        auto configFile = pack.folder / (prefix + "/" + name + ".json");
        std::string parent;
        if (io::exists(configFile)) {
            auto root = io::read_json(configFile);
            root.at("parent").get(parent);
        }
        return parent;
    };
    auto processName = [this](const std::string& name) {
        auto colon = name.find(':');
        auto new_name = name;
        std::string full =
            colon == std::string::npos ? pack.id + ":" + name : name;
        if (colon != std::string::npos) new_name[colon] = '/';

        return std::make_pair(full, new_name);
    };

    for (size_t i = 0; i < defsArr.size(); i++) {
        auto [full, name] = processName(defsArr[i].asString());
        auto parent = getJsonParent(defsDir, name);
        if (parent.empty() || builder.get(parent)) {
            // No dependency or dependency already loaded/exists in another
            // content pack
            bool created;
            auto& def = builder.create(full, &created);
            loadUnit(def, full, name);
            if (postFunc) {
                postFunc(def);
            }
        } else {
            // Dependency not loaded yet, add to pending content units
            pendingDefs.emplace_back(full, name);
        }
    }

    // Resolve dependencies for pending content units
    bool progressMade = true;
    while (!pendingDefs.empty() && progressMade) {
        progressMade = false;

        for (auto it = pendingDefs.begin(); it != pendingDefs.end();) {
            auto parent = getJsonParent(defsDir, it->second);
            if (builder.get(parent)) {
                // Dependency resolved or parent exists in another pack,
                // load the content unit
                bool created;
                auto& def = builder.create(it->first, &created);
                loadUnit(def, it->first, it->second);
                if (postFunc) {
                    postFunc(def);
                }
                it = pendingDefs.erase(it);  // Remove resolved content unit
                progressMade = true;
            } else {
                ++it;
            }
        }
    }

    if (!pendingDefs.empty()) {
        // Handle circular dependencies or missing dependencies
        // You can log an error or throw an exception here if necessary
        throw std::runtime_error(
            "Unresolved " + defsDir + " dependencies detected."
        );
    }
}

void ContentLoader::loadContent(const dv::value& root) {
    ContentPackStats prevStats {
        builder.blocks.defs.size(),
        builder.items.defs.size(),
        builder.entities.defs.size(),
    };

    ContentUnitLoader<Block>(*pack, builder.blocks, "blocks", 
        [this](Block& def) {
        if (!def.hidden) {
            bool created;
            auto& item = builder.items.create(def.name + BLOCK_ITEM_SUFFIX, &created);
            item.generated = true;
            item.caption = def.caption;
            item.iconType = ItemIconType::BLOCK;
            item.icon = def.name;
            item.placingBlock = def.name;
            item.tags = def.tags;
    
            for (uint j = 0; j < 4; j++) {
                item.emission[j] = def.emission[j];
            }
        }
    }).loadDefs(root);

    ContentUnitLoader(*pack, builder.items, "items").loadDefs(root);
    ContentUnitLoader(*pack, builder.entities, "entities").loadDefs(root);

    stats->totalBlocks = builder.blocks.defs.size() - prevStats.totalBlocks;
    stats->totalItems = builder.items.defs.size() - prevStats.totalItems;
    stats->totalEntities = builder.entities.defs.size() - prevStats.totalEntities;
}

static inline void foreach_file(
    const io::path& dir, std::function<void(const io::path&)> handler
) {
    if (!io::is_directory(dir)) {
        return;
    }
    for (const auto& path : io::directory_iterator(dir)) {
        if (io::is_directory(path)) {
            continue;
        }
        handler(path);
    }
}

static std::tuple<std::string, std::string, std::string> create_unit_id(
    const std::string& packid, const std::string& name
) {
    size_t colon = name.find(':');
    if (colon == std::string::npos) {
        return {packid, packid + ":" + name, name};
    }
    auto otherPackid = name.substr(0, colon);
    auto full = otherPackid + ":" + name;
    return {otherPackid, full, otherPackid + "/" + name};
}

void ContentLoader::load() {
    logger.info() << "loading pack [" << pack->id << "]";

    fixPackIndices();

    auto folder = pack->folder;

    builder.defaults = paths.readCombinedObject(
        EnginePaths::CONFIG_DEFAULTS.string()
    );

    // Load world generators
    io::path generatorsDir = folder / "generators";
    foreach_file(generatorsDir, [this](const io::path& file) {
        std::string name = file.stem();
        auto [packid, full, filename] = create_unit_id(pack->id, name);

        auto& def = builder.generators.create(full);
        try {
            loadGenerator(def, full, name);
        } catch (const std::runtime_error& err) {
            throw std::runtime_error("generator '"+full+"': "+err.what());
        }
    });

    // Load pack resources.json
    io::path resourcesFile = folder / "resources.json";
    if (io::exists(resourcesFile)) {
        auto resRoot = io::read_json(resourcesFile);
        for (const auto& [key, arr] : resRoot.asObject()) {
            ResourceType type;
            if (ResourceTypeMeta.getItem(key, type)) {
                loadResources(type, arr);
            } else {
                // Ignore unknown resources
                logger.warning() << "unknown resource type: " << key;
            }
        }
    }

    // Load pack resources aliases
    io::path aliasesFile = folder / "resource-aliases.json";
    if (io::exists(aliasesFile)) {
        auto resRoot = io::read_json(aliasesFile);
        for (const auto& [key, arr] : resRoot.asObject()) {
            ResourceType type;
            if (ResourceTypeMeta.getItem(key, type)) {
                loadResourceAliases(type, arr);
            } else {
                // Ignore unknown resources
                logger.warning() << "unknown resource type: " << key;
            }
        }
    }

    // Load block materials
    io::path materialsDir = folder / "block_materials";    
    if (io::is_directory(materialsDir)) {
        for (const auto& file : io::directory_iterator(materialsDir)) {
            auto [packid, full, filename] =
                create_unit_id(pack->id, file.stem());
            loadBlockMaterial(
                builder.createBlockMaterial(full),
                materialsDir / (filename + ".json")
            );
        }
    }

    // Load skeletons
    io::path skeletonsDir = folder / "skeletons";
    foreach_file(skeletonsDir, [this](const io::path& file) {
        std::string name = pack->id + ":" + file.stem();
        std::string text = io::read_string(file);
        builder.add(
            rigging::SkeletonConfig::parse(text, file.string(), name)
        );
    });

    // Process content.json and load defined content units
    auto contentFile = pack->getContentFile();
    if (io::exists(contentFile)) {
        loadContent(io::read_json(contentFile));
    }
}

template <class T>
static void load_script(const Content& content, T& def) {
    const auto& scriptName = def.scriptFile;
    if (scriptName.empty()) return;
    size_t pos = scriptName.find(':');
    if (pos == std::string::npos) {
        throw std::runtime_error("invalid content unit name");
    }
    const auto runtime = content.getPackRuntime(scriptName.substr(0, pos));
    const auto& pack = runtime->getInfo();
    const auto& folder = pack.folder;
    auto scriptfile = folder / ("scripts/" + def.scriptName + ".lua");
    if (io::is_regular_file(scriptfile)) {
        scripting::load_content_script(
            runtime->getEnvironment(),
            def.name,
            scriptfile,
            def.scriptFile,
            def.rt.funcsset
        );
    }
}

template <class T>
static void load_scripts(const Content& content, ContentUnitDefs<T>& units) {
    for (const auto& [_, def] : units.getDefs()) {
        load_script(content, *def);
    }
}

void ContentLoader::reloadScript(const Content& content, Block& block) {
    load_script(content, block);
}

void ContentLoader::reloadScript(const Content& content, ItemDef& item) {
    load_script(content, item);
}

void ContentLoader::loadWorldScript(ContentPackRuntime& runtime) {
    const auto& pack = runtime.getInfo();
    const auto& folder = pack.folder;
    io::path scriptFile = folder / "scripts/world.lua";
    if (io::is_regular_file(scriptFile)) {
        scripting::load_world_script(
            runtime.getEnvironment(),
            pack.id,
            scriptFile,
            pack.id + ":scripts/world.lua",
            runtime.worldfuncsset
        );
    }
}

void ContentLoader::loadScripts(Content& content) {
    load_scripts(content, content.blocks);
    load_scripts(content, content.items);

    for (const auto& [packid, runtime] : content.getPacks()) {
        const auto& pack = runtime->getInfo();
        const auto& folder = pack.folder;
        
        // Load main world script
        loadWorldScript(*runtime);

        // Load entity components
        io::path componentsDir = folder / "scripts/components";
        foreach_file(componentsDir, [&pack](const io::path& file) {
            auto name = pack.id + ":" + file.stem();
            scripting::load_entity_component(
                name,
                file,
                pack.id + ":scripts/components/" + file.name()
            );
        });
    }
}

void ContentLoader::loadResources(ResourceType type, const dv::value& list) {
    for (size_t i = 0; i < list.size(); i++) {
        builder.resourceIndices[static_cast<size_t>(type)].add(
            pack->id + ":" + list[i].asString(), nullptr
        );
    }
}

void ContentLoader::loadResourceAliases(ResourceType type, const dv::value& aliases) {
    for (const auto& [alias, name] : aliases.asObject()) {
        builder.resourceIndices[static_cast<size_t>(type)].addAlias(
            name.asString(), alias
        );
    }
}
