#include "Panel.hpp"

#include <algorithm>

using namespace gui;

Panel::Panel(GUI& gui, glm::vec2 size, glm::vec4 padding, float interval)
    : BasePanel(gui, size, padding, interval) {
    setColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.75f));
}

Panel::~Panel() = default;

void Panel::setMaxLength(int value) {
    maxLength = value;
}

int Panel::getMaxLength() const {
    return maxLength;
}

void Panel::setMinLength(int value) {
    minLength = value;
}

int Panel::getMinLength() const {
    return minLength;
}

void Panel::setContentSize(const glm::ivec2& contentSize) {
    setSize(glm::vec2(
        glm::max(padding.x + padding.z + contentSize.x, size.x),
        glm::max(padding.y + padding.w + contentSize.y, size.y)
    ));
}

glm::vec2 Panel::getContentSize() const {
    return size - glm::vec2(padding.z + padding.x, padding.w + padding.y);
}

void Panel::cropToContent() {
    if (maxLength > 0.0f) {
        setSize(glm::vec2(
            getSize().x, glm::max(minLength, glm::min(maxLength, actualLength))
        ));
    } else {
        setSize(glm::vec2(getSize().x, glm::max(minLength, actualLength)));
    }
}

void Panel::fullRefresh() {
    refresh();
    cropToContent();
    reposition();
    Container::fullRefresh();
}

void Panel::add(const std::shared_ptr<UINode>& node) {
    node->setResizing(true);
    Container::add(node);
    fullRefresh();
}

void Panel::remove(UINode* node) {
    Container::remove(node);
    fullRefresh();
}

void Panel::refresh() {
    UINode::refresh();

    float x = padding.x;
    float y = padding.y;
    glm::vec2 size = getSize();
    if (orientation == Orientation::vertical) {
        float maxw = size.x;
        for (auto& node : nodes) {
            const glm::vec4 margin = node->getMargin();
            y += margin.y;

            float ex = x + margin.x;
            node->setPos(glm::vec2(ex, y));

            float width = size.x - padding.x - padding.z - margin.x - margin.z;
            if (node->isResizing()) {
                node->setMaxSize({width, node->getMaxSize().y});
                node->setSize(glm::vec2(width, node->getSize().y));
            }
            node->refresh();
            glm::vec2 nodeSize = node->getSize();
            y += nodeSize.y + margin.w + interval;
            maxw = fmax(maxw, ex + nodeSize.x + margin.z + padding.z);
        }
        actualLength = y + padding.w;
    } else {
        float maxh = size.y;
        for (auto& node : nodes) {
            glm::vec2 nodesize = node->getSize();
            const glm::vec4 margin = node->getMargin();
            x += margin.x;
            node->setPos(glm::vec2(x, y + margin.y));
            x += nodesize.x + margin.z + interval;

            node->refresh();
            maxh = fmax(
                maxh, y + margin.y + node->getSize().y + margin.w + padding.w
            );
        }
        actualLength = size.y;
    }
}
