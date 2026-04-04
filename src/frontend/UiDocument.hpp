#pragma once

#include "typedefs.hpp"

#include <string>
#include <memory>
#include <unordered_map>

#include "io/fwd.hpp"

namespace gui {
    class GUI;
    class UINode;
}

struct UiDocScript {
    bool onopen : 1;
    bool onprogress : 1;
    bool onclose : 1;
};

using UINodesMap = std::unordered_map<std::string, std::weak_ptr<gui::UINode>>;

class UiDocument {
    std::string id;
    UiDocScript script;
    UINodesMap map;
    std::shared_ptr<gui::UINode> root;
    scriptenv env;
public:
    UiDocument(
        std::string id, 
        UiDocScript script, 
        const std::shared_ptr<gui::UINode> &root,
        scriptenv env
    );

    void rebuildIndices();
    void pushIndices(const std::shared_ptr<gui::UINode>& node);

    const std::string& getId() const;
    const UINodesMap& getMap() const;
    std::shared_ptr<gui::UINode> getRoot() const;
    std::shared_ptr<gui::UINode> get(const std::string& id) const;
    const UiDocScript& getScript() const;
    scriptenv getEnvironment() const;

    static std::unique_ptr<UiDocument> read(
        gui::GUI&,
        const scriptenv& parent_env,
        const std::string& name,
        const io::path& file,
        const std::string& fileName
    );
    static std::shared_ptr<gui::UINode> readElement(
        gui::GUI&, const io::path& file, const std::string& fileName
    );
};
