#include "UiDocument.hpp"

#include <utility>

#include "io/io.hpp"
#include "graphics/ui/elements/UINode.hpp"
#include "graphics/ui/elements/InventoryView.hpp"
#include "graphics/ui/gui_xml.hpp"
#include "logic/scripting/scripting.hpp"

UiDocument::UiDocument(
    std::string id, 
    uidocscript script, 
    const std::shared_ptr<gui::UINode>& root,
    scriptenv env
) : id(std::move(id)), script(script), root(root), env(std::move(env)) {
    rebuildIndices();
}

void UiDocument::rebuildIndices() {
    map.clear();
    gui::UINode::getIndices(root, map);
    map["root"] = root;
}

const UINodesMap& UiDocument::getMap() const {
    return map;
}

UINodesMap& UiDocument::getMapWriteable() {
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
    return found->second;
}

const uidocscript& UiDocument::getScript() const {
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
    const std::string& fileName
) {
    const std::string text = io::read_string(file);
    auto xmldoc = xml::parse(file.string(), text);

    auto env = penv == nullptr 
        ? scripting::create_doc_environment(scripting::get_root_environment(), name)
        : scripting::create_doc_environment(penv, name);

    gui::UiXmlReader reader(gui, scriptenv(env));
    auto view = reader.readXML(file.string(), *xmldoc->getRoot());
    view->setId("root");
    uidocscript script {};
    auto scriptFile = io::path(file.string()+".lua");
    if (io::is_regular_file(scriptFile)) {
        scripting::load_layout_script(
            env, name, scriptFile, fileName + ".lua", script
        );
    }
    return std::make_unique<UiDocument>(name, script, view, env);
}

std::shared_ptr<gui::UINode> UiDocument::readElement(
    gui::GUI& gui, const io::path& file, const std::string& fileName
) {
    return read(gui, nullptr, file.name(), file, fileName)->getRoot();
}
