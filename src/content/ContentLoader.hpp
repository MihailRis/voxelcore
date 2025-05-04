#pragma once

#include <memory>
#include <string>

#include "io/io.hpp"
#include "content_fwd.hpp"
#include "data/dv.hpp"

class Block;
struct BlockMaterial;
struct ItemDef;
struct EntityDef;
struct ContentPack;
struct GeneratorDef;

class ResPaths;
class Content;
class ContentBuilder;
class ContentPackRuntime;
struct ContentPackStats;

class ContentLoader {
    const ContentPack* pack;
    ContentPackRuntime* runtime;
    scriptenv env;
    ContentBuilder& builder;
    ContentPackStats* stats;
    const ResPaths& paths;

    void loadGenerator(
        GeneratorDef& def, const std::string& full, const std::string& name
    );
    static void loadBlockMaterial(BlockMaterial& def, const io::path& file);
    void loadResources(ResourceType type, const dv::value& list);
    void loadResourceAliases(ResourceType type, const dv::value& aliases);

    void loadContent(const dv::value& map);
public:
    ContentLoader(
        ContentPack* pack,
        ContentBuilder& builder,
        const ResPaths& paths
    );

    // Refresh pack content.json
    static bool fixPackIndices(
        const io::path& folder,
        dv::value& indicesRoot,
        const std::string& contentSection
    );

    static std::vector<std::tuple<std::string, std::string>> scanContent(
        const ContentPack& pack, ContentType type
    );

    void fixPackIndices();
    void load();

    static void loadScripts(Content& content);
    static void loadWorldScript(ContentPackRuntime& pack);
    static void reloadScript(const Content& content, Block& block);
    static void reloadScript(const Content& content, ItemDef& item);
};
