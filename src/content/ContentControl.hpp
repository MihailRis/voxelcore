#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>

#include "io/path.hpp"
#include "ContentPack.hpp"

class Content;
class PacksManager;
class EnginePaths;
class Input;
struct Project;

class ContentControl {
public:
    ContentControl(
        const Project& project,
        EnginePaths& paths,
        Input& input,
        std::function<void()> postContent
    );
    ~ContentControl();

    Content* get();

    const Content* get() const;

    std::vector<std::string>& getBasePacks();

    /// @brief Reset content to base packs list
    void resetContent(const std::vector<std::string>& nonReset);

    void loadContent(const std::vector<std::string>& names);

    void loadContent();

    void setContentPacksRaw(std::vector<ContentPack>&& packs);

    const std::vector<ContentPack>& getContentPacks() const;
    const std::vector<ContentPack>& getAllContentPacks() const;

    PacksManager& scan();

    void setContentSources(std::vector<io::path> sources);
    void resetContentSources();
    const std::vector<io::path>& getContentSources() const;
private:
    EnginePaths& paths;
    Input& input;
    std::unique_ptr<Content> content;
    std::function<void()> postContent;
    std::vector<std::string> basePacks;
    std::unique_ptr<PacksManager> manager;
    std::vector<ContentPack> contentPacks;
    std::vector<ContentPack> allPacks; // includes 'core'
};
