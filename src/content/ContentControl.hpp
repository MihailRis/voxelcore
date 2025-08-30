#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>

#include "ContentPack.hpp"

class Content;
class PacksManager;
class EnginePaths;
class Input;
struct Project;

namespace io {
    class path;
}

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
    void resetContent();

    void loadContent(const std::vector<std::string>& names);

    void loadContent();

    void setContentPacksRaw(std::vector<ContentPack>&& packs);

    const std::vector<ContentPack>& getContentPacks() const;
    const std::vector<ContentPack>& getAllContentPacks() const;

    PacksManager& scan();
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
