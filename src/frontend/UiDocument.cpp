#include "UiDocument.hpp"

#include <utility>

#include "io/io.hpp"
#include "graphics/ui/elements/UINode.hpp"
#include "graphics/ui/elements/InventoryView.hpp"
#include "graphics/ui/gui_xml.hpp"
#include "logic/scripting/scripting.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("ui-document");

UiDocument::UiDocument(
    std::string id, 
    UiDocScript script, 
    const std::shared_ptr<gui::UINode>& root,
    scriptenv env
) : id(std::move(id)), script(script), root(root), env(std::move(env)) {
    rebuildIndices();
}

UiDocument::~UiDocument() {
    try {
        scripting::on_ui_destroy(*this);
    } catch (const std::exception& err) {
        logger.error() << "an error occurred on calling on_destroy event for document '"
          << id << "': " << err.what();
    } catch (...) {
        logger.error() << "unknown exception caught on calling on_destroy "
                          "event for document '"
                       << id << "'";
    }
}

void UiDocument::rebuildIndices() {
    map.clear();
    gui::UINode::getIndices(root, map);
    map["root"] = root;
}

void UiDocument::pushIndices(const std::shared_ptr<gui::UINode>& node) {
    gui::UINode::getIndices(node, map);
    map["root"] = root;
}

const UINodesMap& UiDocument::getMap() const {
    return map;
}

const std::string& UiDocument::getId() const {
    return id;
}

std::shared_ptr<gui::UINode> UiDocument::getRoot() const {
    return root;
}

std::shared_ptr<gui::UINode> UiDocument::get(const std::string& id) const {
    auto found = map.find(id);
    if (found == map.end()) {
        return nullptr;
    }
    return found->second.lock();
}

const UiDocScript& UiDocument::getScript() const {
    return script;
}

scriptenv UiDocument::getEnvironment() const {
    return env;
}

std::unique_ptr<UiDocument> UiDocument::read(
    gui::GUI& gui,
    const scriptenv& penv,
    const std::string& name,
    const io::path& file,
    const std::string& fileName,
    scriptenv&& env
) {
    const std::string text = io::read_string(file);
    auto xmldoc = xml::parse(file.string(), text);

    if (env == nullptr) {
        env = penv == nullptr 
            ? scripting::create_doc_environment(scripting::get_root_environment(), name)
            : scripting::create_doc_environment(penv, name);
    }

    gui::UiXmlReader reader(gui, env);
    auto view = reader.readXML(file.string(), *xmldoc->getRoot());
    view->setId("root");
    UiDocScript script {};
    auto scriptFile = io::path(file.string()+".lua");
    if (io::is_regular_file(scriptFile)) {
        scripting::load_layout_script(
            env, name, scriptFile, fileName + ".lua", script
        );
    }
    return std::make_unique<UiDocument>(name, script, view, env);
}
