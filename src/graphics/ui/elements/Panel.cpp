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
            glm::max(minLength, glm::min(maxLength, actualLengthX)),
            glm::max(minLength, glm::min(maxLength, actualLengthY))
        ));
    } else {
        setSize(glm::vec2(
            glm::max(minLength, actualLengthX),
            glm::max(minLength, actualLengthY)
        ));
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
    Container::refresh();

    float x = padding.x;
    float y = padding.y;
    glm::vec2 size = getSize();
    if (orientation == Orientation::VERTICAL) {
        float maxw = size.x;
        for (size_t i = 0; i < nodes.size(); i++) {
            const auto& node = nodes[i];
            const glm::vec4 margin = node->getMargin();
            y += margin.y + (i > 0 ? interval : 0);

            float ex = x + margin.x;
            node->setPos(glm::vec2(ex, y));

            int width = glm::floor(
                size.x - padding.x - padding.z - margin.x - margin.z
            );
            if (node->isResizing()) {
                node->setMaxSize({width, node->getMaxSize().y});
                node->setSize(glm::vec2(width, node->getSize().y));
            }
            node->refresh();
            glm::vec2 nodeSize = node->getSize();
            y += nodeSize.y + margin.w;
            maxw = fmax(maxw, ex + nodeSize.x + margin.z + padding.z);
        }
        actualLengthX = size.x;
        actualLengthY = y + padding.w;
    } else {
        float maxh = size.y;
        for (size_t i = 0; i < nodes.size(); i++) {
            const auto& node = nodes[i];
            const glm::vec4 margin = node->getMargin();
            x += margin.x + (i > 0 ? interval : 0);

            float ey = y + margin.y;
            node->setPos(glm::vec2(x, ey));

            int height = glm::floor(
                size.y - padding.y - padding.w - margin.y - margin.w
            );
            if (node->isResizing()) {
                node->setMaxSize({node->getMaxSize().x, height});
                node->setSize(glm::vec2(node->getSize().x, height));
            }
            node->refresh();
            glm::vec2 nodesize = node->getSize();
            x += nodesize.x + margin.z;
            maxh = fmax(maxh, ey + nodesize.y + margin.w + padding.w);
        }
        actualLengthY = size.y;
        actualLengthX = x + padding.z;
    }
}
