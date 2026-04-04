#include "GUI.hpp"

#include <algorithm>
#include <utility>

#include "assets/Assets.hpp"
#include "debug/Logger.hpp"
#include "elements/Label.hpp"
#include "elements/Menu.hpp"
#include "elements/Panel.hpp"
#include "elements/Frame.hpp"
#include "elements/UINode.hpp"
#include "engine/Engine.hpp"
#include "frontend/UiDocument.hpp"
#include "frontend/locale.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Shader.hpp"
#include "gui_util.hpp"
#include "window/Camera.hpp"
#include "window/Window.hpp"
#include "window/input.hpp"

#include <algorithm>
#include <utility>

static debug::Logger logger("gui");

using namespace gui;

GUI::GUI(Engine& engine)
    : engine(engine),
      input(engine.getInput()),
      batch2D(std::make_unique<Batch2D>(1024)),
      container(std::make_shared<Frame>(*this, CORE_MAIN, "")) {
    container->setId("root");
    uicamera =
        std::make_unique<Camera>(glm::vec3(), engine.getWindow().getSize().y);
    uicamera->perspective = false;
    uicamera->flipped = true;
    uicamera->near = -1.0f;
    uicamera->far = 1.0f;

    menu = std::make_shared<Menu>(*this);
    menu->setId("menu");
    menu->setZIndex(10);
    container->add(menu);
    container->setScrollable(false);

    tooltip = guiutil::create(
        *this,
        "<container color='#000000A0' interactive='false' z-index='999'>"
            "<label id='tooltip.label' markup='md' pos='2' autoresize='true' multiline='true' text-wrap='false'></label>"
        "</container>"
    );
    store("tooltip", tooltip);
    store("tooltip.label", UINode::find(tooltip, "tooltip.label"));
    container->add(tooltip);

    rootDocument = std::make_unique<UiDocument>(
        "core:root",
        UiDocScript {},
        std::dynamic_pointer_cast<gui::UINode>(container),
        nullptr
    );
    addFrame(container);
    activeFrame = container;
}

GUI::~GUI() = default;

std::shared_ptr<Menu> GUI::getMenu() {
    return menu;
}

void GUI::setSyntaxColorScheme(std::unique_ptr<FontStylesScheme> scheme) {
    syntaxColorScheme = std::move(scheme);
}

FontStylesScheme* GUI::getSyntaxColorScheme() const {
    return syntaxColorScheme.get();
}

void GUI::onAssetsLoad(Assets* assets) {
    rootDocument->rebuildIndices();
    assets->store(rootDocument, "core:root");
}

void GUI::resetTooltip() {
    tooltipTimer = 0.0f;
    tooltip->setVisible(false);
}

void GUI::updateTooltip(float delta) {
    const auto& cursor = input.getCursor();
    if (hover == nullptr || !hover->isInside(cursor.pos)) {
        return resetTooltip();
    }
    if (tooltipTimer + delta >= hover->getTooltipDelay()) {
        auto label =
            std::dynamic_pointer_cast<gui::Label>(get("tooltip.label"));
        const auto& text = hover->getTooltip();
        if (text.empty() && tooltip->isVisible()) {
            return resetTooltip();
        }
        if (label && !text.empty()) {
            tooltip->setVisible(true);
            label->setText(langs::get(text));
            auto size = label->getSize() + glm::vec2(4.0f);
            auto pos = cursor.pos + glm::vec2(10.0f);
            auto rootSize = container->getSize();
            pos.x = glm::min(pos.x, rootSize.x - size.x);
            pos.y = glm::min(pos.y, rootSize.y - size.y);
            tooltip->setSize(size);
            tooltip->setPos(pos);
        }
    }
    tooltipTimer += delta;
}

/// @brief Mouse related input and logic handling
void GUI::actMouse(Frame& frame, float delta, const CursorState& cursor) {
    float mouseDelta = glm::length(cursor.delta);
    doubleClicked = false;
    doubleClickTimer += delta + mouseDelta * 0.1f;

    auto cursorPos = cursor.pos;
    if (cursorLocator) {
        cursorPos = cursorLocator();
        if (activeFrame) {
            cursorPos.y += activeFrame->getPos().y;
        }
    }
    auto hover = frame.getAt(cursorPos);
    if (this->hover && this->hover != hover) {
        this->hover->setMouseEnter(false);
    }
    if (hover) {
        if (hover != this->hover) {
            hover->setMouseEnter(true);
        }
        int scroll = input.getScroll();
        if (scroll) {
            hover->scrolled(scroll);
        }
    }
    this->hover = hover;
    auto node = hover;

    while (node) {
        if (std::find_if(
                mouseOver.begin(),
                mouseOver.end(),
                [&hover](const std::weak_ptr<UINode>& weak) {
                    auto locked = weak.lock();
                    return locked && locked == hover;
                }) != mouseOver.end()) {
            break;
        }
        mouseOver.push_back(node);
        node->setMouseOver(true);
        auto parent = node->getParent();
        if (parent) {
            node = parent->shared_from_this();
        }
    }

    for (auto it = mouseOver.begin(); it != mouseOver.end(); ) {
        auto mouseOverNode = it->lock();
        if (mouseOverNode) {
            if (mouseOverNode->isInside(cursorPos)) {
                ++it;
                continue;
            }
            mouseOverNode->setMouseOver(false);
        }
        it = mouseOver.erase(it);
    }

    if (input.jclicked(Mousecode::BUTTON_1)) {
        if (pressed == nullptr && this->hover) {
            pressed = hover;
            if (doubleClickTimer < doubleClickDelay) {
                pressed->doubleClick(cursorPos.x, cursorPos.y);
                doubleClicked = true;
            } else {
                pressed->click(cursorPos.x, cursorPos.y);
            }
            doubleClickTimer = 0.0f;
            if (focus && focus != pressed) {
                focus->defocus();
            }
            if (focus != pressed) {
                focus = pressed;
                focus->onFocus();
                return;
            }
        }
        if (this->hover == nullptr && focus) {
            focus->defocus();
            focus = nullptr;
        }
    } else if (!input.clicked(Mousecode::BUTTON_1) && pressed) {
        pressed->mouseRelease(cursorPos.x, cursorPos.y);
        pressed = nullptr;
    }

    if (hover) {
        for (Mousecode code : MOUSECODES_ALL) {
            if (input.jclicked(code)) {
                hover->clicked(code);
            }
        }
    }
}

void GUI::actFocused() {
    if (input.jpressed(Keycode::ESCAPE)) {
        focus->defocus();
        focus = nullptr;
        return;
    }
    for (auto codepoint : input.getCodepoints()) {
        focus->typed(codepoint);
    }
    for (auto key : input.getPressedKeys()) {
        focus->keyPressed(key);
    }

    const auto& cursor = input.getCursor();
    if (!cursor.locked) {
        if (input.clicked(Mousecode::BUTTON_1) &&
            (input.jclicked(Mousecode::BUTTON_1) || cursor.delta.x ||
             cursor.delta.y)) {
            if (!doubleClicked) {
                focus->mouseMove(cursor.pos.x, cursor.pos.y);
            }
        }
    }
}

void GUI::act(float delta, const glm::uvec2& vp) {
    container->setSize(vp);
    for (auto& pair : frames) {
        pair.second->act(delta);
    }
    auto prevfocus = focus;

    updateTooltip(delta);

    const auto& cursor = input.getCursor();
    if (!cursor.locked && activeFrame) {
        actMouse(*activeFrame, delta, cursor);
    } else {
        if (hover) {
            hover->setMouseEnter(false);
            hover = nullptr;
        }
    }
    if (focus) {
        actFocused();
    }
    if (focus && !focus->isFocused()) {
        focus = nullptr;
    }
}

void GUI::postAct() {
    while (!postRunnables.empty()) {
        runnable callback = postRunnables.front();
        postRunnables.pop();
        callback();
    }
}

void GUI::draw(const DrawContext& pctx, Assets& assets) {
    auto ctx = pctx.sub(batch2D.get());

    auto& viewport = ctx.getViewport();

    auto& page = menu->getCurrent();
    if (page.panel) {
        menu->setSize(page.panel->getSize());
        page.panel->refresh();
        if (auto panel = std::dynamic_pointer_cast<gui::Panel>(page.panel)) {
            panel->cropToContent();
        }
    }
    menu->setPos((glm::vec2(viewport) - menu->getSize()) / 2.0f);
    uicamera->setFov(viewport.y);
    uicamera->setAspectRatio(viewport.x / static_cast<float>(viewport.y));

    auto uishader = assets.get<Shader>("ui");
    uishader->use();
    uishader->uniformMatrix("u_projview", uicamera->getProjView());

    batch2D->begin();
    for (auto& [outputTexture, frame] : frames) {
        frame->updateOutput(assets);
        frame->draw(ctx, assets);
    }

    if (hover) {
        engine.getWindow().setCursor(hover->getCursor());
    }
    if (hover && debug) {
        auto pos = hover->calcPos();
        const auto& id = hover->getId();
        if (!id.empty()) {
            auto& font = assets.require<Font>(FONT_DEFAULT);
            auto text = util::str2wstr_utf8(id);
            int width = font.calcWidth(text);
            int height = font.getLineHeight();

            batch2D->untexture();
            batch2D->setColor(0, 0, 0);
            batch2D->rect(pos.x, pos.y, width, height);

            batch2D->resetColor();
            font.draw(*batch2D, text, pos.x, pos.y, nullptr, 0);
        }

        batch2D->untexture();
        auto node = hover->getParent();
        while (node) {
            auto parentPos = node->calcPos();
            auto size = node->getSize();

            batch2D->setColor(0, 255, 255);
            batch2D->lineRect(parentPos.x+1, parentPos.y, size.x-2, size.y-1);

            node = node->getParent();
        }
        // debug draw
        auto size = hover->getSize();
        batch2D->setColor(0, 255, 0);
        batch2D->lineRect(pos.x, pos.y, size.x-1, size.y-1);
    }
}

std::shared_ptr<UINode> GUI::getFocused() const {
    return focus;
}

bool GUI::isFocusCaught() const {
    return focus && focus->isFocuskeeper();
}

void GUI::add(std::shared_ptr<UINode> node) {
    rootDocument->pushIndices(node);
    container->add(std::move(node));
}

void GUI::addFrame(std::shared_ptr<Frame> frame) {
    const auto& id = frame->getFrameId();
    frames[id] = std::move(frame);
}

void GUI::setActiveFrame(const std::string& id, vec2supplier cursorLocator) {
    this->cursorLocator = std::move(cursorLocator);
    if (id.empty()) {
        activeFrame = nullptr;
        return;
    }
    const auto& found = frames.find(id);
    if (found == frames.end()) {
        logger.error() << "attempted to make non-existing frame '" << id
                       << "' as active";
        return;
    }
    activeFrame = found->second;
}

std::shared_ptr<gui::Frame> GUI::getActiveFrame() const {
    return activeFrame;
}

void GUI::remove(UINode* node) noexcept {
    container->remove(node);
}

void GUI::store(const std::string& name, std::shared_ptr<UINode> node) {
    storage[name] = std::move(node);
}

std::shared_ptr<UINode> GUI::get(const std::string& name) noexcept {
    auto found = storage.find(name);
    if (found == storage.end()) {
        return nullptr;
    }
    return found->second;
}

void GUI::remove(const std::string& name) noexcept {
    storage.erase(name);
}

void GUI::setFocus(std::shared_ptr<UINode> node) {
    if (focus) {
        focus->defocus();
    }
    focus = std::move(node);
    if (focus) {
        focus->onFocus();
    }
}

std::shared_ptr<Container> GUI::getContainer() const {
    return container;
}

void GUI::postRunnable(const runnable& callback) {
    postRunnables.push(callback);
}

void GUI::setDoubleClickDelay(float delay) {
    doubleClickDelay = delay;
}

float GUI::getDoubleClickDelay() const {
    return doubleClickDelay;
}

void GUI::toggleDebug() {
    debug = !debug;
}

const Input& GUI::getInput() const {
    return engine.getInput();
}

Input& GUI::getInput() {
    return engine.getInput();
}

Window& GUI::getWindow() {
    return engine.getWindow();
}

devtools::Editor& GUI::getEditor() {
    return engine.getEditor();
}

Engine& GUI::getEngine() {
    return engine;
}
