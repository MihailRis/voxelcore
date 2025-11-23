#include "Container.hpp"

#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Batch2D.hpp"
#include "window/Window.hpp"
#include "../GUI.hpp"

#include <algorithm>
#include <utility>

using namespace gui;

Container::Container(GUI& gui, glm::vec2 size) : UINode(gui, size) {
    actualLength = size.y;
    setColor(glm::vec4());
}

Container::~Container() {
    Container::clear();
}

std::shared_ptr<UINode> Container::getAt(const glm::vec2& pos) {
    if (!isInteractive() || !isEnabled()) {
        return nullptr;
    }
    if (!isInside(pos)) {
        return nullptr;
    }
    int diff = (actualLength-size.y);
    if (scrollable && diff > 0 && pos.x > calcPos().x + getSize().x - scrollBarWidth) {
        return UINode::getAt(pos);
    }

    for (int i = nodes.size()-1; i >= 0; i--) {
        auto& node = nodes[i];
        if (!node->isVisible())
            continue;
        auto hover = node->getAt(pos);
        if (hover != nullptr) {
            return hover;
        }
    }
    return UINode::getAt(pos);
}

void Container::mouseMove(int x, int y) {
    UINode::mouseMove(x, y);
    if (!scrollable) {
        return;
    }
    auto pos = calcPos();
    x -= pos.x;
    y -= pos.y;
    if (prevScrollY == -1) {
        if (x >= size.x - scrollBarWidth) {
            prevScrollY = y;
        }
        return;
    }
    int diff = (actualLength-size.y);
    if (diff > 0) {
        scroll -= (y - prevScrollY) / static_cast<float>(size.y) * actualLength;
        scroll = -glm::min(
            glm::max(static_cast<float>(-scroll), 0.0f), actualLength - size.y
        );
    }
    prevScrollY = y;
}

void Container::mouseRelease(int x, int y) {
    UINode::mouseRelease(x, y);
    prevScrollY = -1;
}

void Container::act(float delta) {
    UINode::act(delta);
    for (const auto& node : nodes) {
        if (node->isVisible()) {
            node->act(delta);
        }
    }
    if (!intervalEvents.empty()) {
        // TODO: make it interval-based
        gui.getWindow().setShouldRefresh();
    }
    for (IntervalEvent& event : intervalEvents) {
        event.timer += delta;
        if (event.timer > event.interval) {
            event.callback();
            event.timer = fmod(event.timer, event.interval);
            if (event.repeat > 0) {
                event.repeat--;
            }
        }
    }
    intervalEvents.erase(std::remove_if(
        intervalEvents.begin(), intervalEvents.end(),
        [](const IntervalEvent& event) {
            return event.repeat == 0;
        }
    ), intervalEvents.end());
}

void Container::scrolled(int value) {
    int diff = (actualLength-getSize().y);
    if (scroll < 0 && diff <= 0) {
        scroll = 0;
    }
    if (diff > 0 && scrollable) {
        scroll += value * scrollStep;
        if (scroll > 0)
            scroll = 0;
        if (-scroll > diff) {
            scroll = -diff;
        }
    } else if (parent) {
        parent->scrolled(value);
    }
}

void Container::setScrollable(bool flag) {
    scrollable = flag;
}

void Container::draw(const DrawContext& pctx, const Assets& assets) {
    glm::vec2 pos = calcPos();
    glm::vec2 size = getSize();
    drawBackground(pctx, assets);

    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);
    if (!nodes.empty()) {
        batch->flush();
        DrawContext ctx = pctx.sub();
        ctx.setScissors(glm::vec4(pos.x, pos.y, glm::ceil(size.x), glm::ceil(size.y)));
        for (const auto& node : nodes) {
            if (node->isVisible())
                node->draw(pctx, assets);
        }

        int diff = (actualLength-size.y);
        if (scrollable && diff > 0) {
            int h = glm::max(size.y / actualLength * size.y, scrollBarWidth / 2.0f);
            batch->untexture();
            batch->setColor(glm::vec4(1, 1, 1, 0.3f));
            batch->rect(
                pos.x + size.x - scrollBarWidth,
                pos.y - scroll / static_cast<float>(diff) * (size.y - h),
                scrollBarWidth, h
            );
        }
        batch->flush();
    }
}

void Container::drawBackground(const DrawContext& pctx, const Assets&) {
    glm::vec4 color = calcColor();
    if (color.a <= 0.001f)
        return;
    glm::vec2 pos = calcPos();

    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);
    batch->setColor(color);
    batch->rect(pos.x, pos.y, glm::ceil(size.x), glm::ceil(size.y));
}

void Container::add(const std::shared_ptr<UINode>& node) {
    nodes.push_back(node);
    node->setParent(this);
    node->reposition();
    mustRefresh = true;

    auto parent = getParent();
    while (parent) {
        parent->setMustRefresh();
        parent = parent->getParent();
    }
    gui.getWindow().setShouldRefresh();
}

void Container::add(const std::shared_ptr<UINode>& node, glm::vec2 pos) {
    node->setPos(pos);
    add(node);
}

void Container::remove(UINode* selected) {
    selected->setParent(nullptr);
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(), 
        [selected](const std::shared_ptr<UINode>& node) {
            return node.get() == selected;
        }
    ), nodes.end());
    refresh();
}

void Container::remove(const std::string& id) {
    for (auto& node : nodes) {
        if (node->getId() == id) {
            return remove(node.get());
        }
    }
}

void Container::clear() {
    for (const auto& node : nodes) {
        node->setParent(nullptr);
    }
    nodes.clear();
    refresh();
}

void Container::listenInterval(float interval, OnTimeOut callback, int repeat) {
    intervalEvents.push_back({std::move(callback), interval, 0.0f, repeat});
}

void Container::setSize(const glm::vec2& size) {
    if (size == getSize()) {
        return;
    }
    UINode::setSize(size);
    refresh();
    for (auto& node : nodes) {
        node->reposition();
    }
}

int Container::getScrollStep() const {
    return scrollStep;
}

void Container::setScrollStep(int step) {
    scrollStep = step;
}

void Container::refresh() {
    std::stable_sort(nodes.begin(), nodes.end(), [](const auto& a, const auto& b) {
        return a->getZIndex() < b->getZIndex();
    });
}

void Container::setScroll(int scroll) {
    this->scroll = scroll;
}

const std::vector<std::shared_ptr<UINode>>& Container::getNodes() const {
    return nodes;
}
