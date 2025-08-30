#include "InlineFrame.hpp"
#include "frontend/UiDocument.hpp"
#include "logic/scripting/scripting.hpp"
#include "assets/Assets.hpp"
#include "engine/Engine.hpp"
#include "../GUI.hpp"

using namespace gui;

InlineFrame::InlineFrame(GUI& gui) : Container(gui, glm::vec2(1)) {}
InlineFrame::~InlineFrame() = default;

void InlineFrame::setSrc(const std::string& src) {
    this->src = src;
    if (document) {
        scripting::on_ui_close(document.get(), nullptr);
        document = nullptr;
        root = nullptr;
    }
}

void InlineFrame::setDocument(const std::shared_ptr<UiDocument>& document) {
    clear();
    if (document == nullptr) {
        return;
    }
    this->document = document;
    this->root = document->getRoot();
    add(root);

    root->setSize(size);

    gui.postRunnable([this]() {
        scripting::on_ui_open(this->document.get(), {});
    });
}

void InlineFrame::act(float delta) {
    Container::act(delta);
    if (document || src.empty()) {
        return;
    }
    const auto& assets = *gui.getEngine().getAssets();
    setDocument(assets.getShared<UiDocument>(src));
}

void InlineFrame::setSize(glm::vec2 size) {
    Container::setSize(size);
    if (root) {
        root->setSize(size);
    }
}

const std::string& InlineFrame::getSrc() const {
    return src;
}
