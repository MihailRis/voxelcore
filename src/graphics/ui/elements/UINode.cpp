#include "UINode.hpp"

#include <utility>

#include "Container.hpp"
#include "graphics/core/Batch2D.hpp"

using gui::UINode;
using gui::Align;

UINode::UINode(GUI& gui, glm::vec2 size) : gui(gui), size(size) {
}

UINode::~UINode() {
}

bool UINode::isVisible() const {
    if (visible && parent) {
        return parent->isVisible();
    }
    return visible;
}

void UINode::setVisible(bool flag) {
    visible = flag;
}

void UINode::setEnabled(bool flag) {
    enabled = flag;
    if (!flag) {
        defocus();
        hover = false;
    }
}

bool UINode::isEnabled() const {
    if (enabled && parent) {
        return parent->isEnabled();
    }
    return enabled;
}

Align UINode::getAlign() const {
    return align;
}

void UINode::setAlign(Align align) {
    this->align = align;
}

void UINode::setHover(bool flag) {
    hover = flag;
}

bool UINode::isHover() const {
    return hover;
}

void UINode::setParent(UINode* node) {
    parent = node;
}

UINode* UINode::getParent() const {
    return parent;
}

void UINode::listenClick(OnAction action) {
    actions.listen(UIAction::CLICK, std::move(action));
}

void UINode::listenRightClick(OnAction action) {
    actions.listen(UIAction::RIGHT_CLICK, std::move(action));
}

void UINode::listenDoubleClick(OnAction action) {
    actions.listen(UIAction::DOUBLE_CLICK, std::move(action));
}

void UINode::listenFocus(OnAction action) {
    actions.listen(UIAction::FOCUS, std::move(action));
}

void UINode::listenDefocus(OnAction action) {
    actions.listen(UIAction::DEFOCUS, std::move(action));
}

void UINode::click(int, int) {
    pressed = true;
}

void UINode::clicked(Mousecode button) {
    if (button == Mousecode::BUTTON_2) {
        actions.notify(UIAction::RIGHT_CLICK, gui);
    }
}

void UINode::doubleClick(int x, int y) {
    pressed = true;
    if (isInside(glm::vec2(x, y))) {
        actions.notify(UIAction::DOUBLE_CLICK, gui);
    }
}

void UINode::mouseRelease(int x, int y) {
    pressed = false;
    if (isInside(glm::vec2(x, y))) {
        actions.notify(UIAction::CLICK, gui);
    }
}

bool UINode::isPressed() const {
    return pressed;
}

void UINode::onFocus() {
    focused = true;
    actions.notify(UIAction::FOCUS, gui);
}

void UINode::defocus() {
    focused = false;
    actions.notify(UIAction::DEFOCUS, gui);
}

bool UINode::isFocused() const {
    return focused;
}

bool UINode::isInside(glm::vec2 point) {
    glm::vec2 pos = calcPos();
    glm::vec2 size = getSize();
    return (point.x >= pos.x && point.y >= pos.y && 
            point.x < pos.x + size.x && point.y < pos.y + size.y);
}

std::shared_ptr<UINode> UINode::getAt(const glm::vec2& point) {
    if (!isInteractive() || !enabled) {
        return nullptr;
    }
    return isInside(point) ? shared_from_this() : nullptr;
}

bool UINode::isInteractive() const {
    return interactive && isVisible();
}

void UINode::setInteractive(bool flag) {
    interactive = flag;
}

void UINode::setResizing(bool flag) {
    resizing = flag;
}

bool UINode::isResizing() const {
    return resizing;
}

void UINode::setTooltip(const std::wstring& text) {
    this->tooltip = text;
}

const std::wstring& UINode::getTooltip() const {
    return tooltip;
}

void UINode::setTooltipDelay(float delay) {
    tooltipDelay = delay;
}

float UINode::getTooltipDelay() const {
    return tooltipDelay;
}

void UINode::setCursor(CursorShape shape) {
    cursor = shape;
}

CursorShape UINode::getCursor() const {
    return cursor;
}

glm::vec2 UINode::calcPos() const {
    if (parent) {
        return glm::ivec2(pos + parent->calcPos() + parent->getContentOffset());
    }
    return glm::ivec2(pos);
}

void UINode::scrolled(int value) {
    if (parent) {
        parent->scrolled(value);
    }
}

glm::vec4 UINode::calcColor() const {
    glm::vec4 color = this->color;
    if (isEnabled()) {
        color = (isPressed() ? pressedColor : (hover ? hoverColor : color));
    } else {
        color = glm::vec4(color.r, color.g, color.b, color.a * 0.5f);
    }
    return color;
}

void UINode::setPos(const glm::vec2& pos) {
    this->pos = pos;
}

glm::vec2 UINode::getPos() const {
    return pos;
}

glm::vec2 UINode::getSize() const {
    return size;
}

void UINode::setSize(const glm::vec2& size) {
    this->size = glm::vec2(
        glm::max(minSize.x, glm::min(maxSize.x, size.x)),
        glm::max(minSize.y, glm::min(maxSize.y, size.y))
    );
}

glm::vec2 UINode::getMinSize() const {
    return minSize;
}

void UINode::setMinSize(const glm::vec2& minSize) {
    this->minSize = minSize;
    setSize(getSize());
}

glm::vec2 UINode::getMaxSize() const {
    return maxSize;
}

void UINode::setMaxSize(const glm::vec2& maxSize) {
    this->maxSize = maxSize;
    setSize(getSize());
}

void UINode::setColor(glm::vec4 color) {
    this->color = color;
    this->hoverColor = color;
    this->pressedColor = color;
}

void UINode::setHoverColor(glm::vec4 newColor) {
    this->hoverColor = newColor;
}

glm::vec4 UINode::getHoverColor() const {
    return hoverColor;
}

glm::vec4 UINode::getColor() const {
    return color;
}

glm::vec4 UINode::getPressedColor() const {
    return pressedColor;
}

void UINode::setPressedColor(glm::vec4 color) {
    pressedColor = color;
}

void UINode::setMargin(glm::vec4 margin) {
    this->margin = margin;
}

glm::vec4 UINode::getMargin() const {
    return margin;
}

void UINode::setZIndex(int zindex) {
    this->zindex = zindex;
}

int UINode::getZIndex() const {
    return zindex;
}

void UINode::moveInto(
    const std::shared_ptr<UINode>& node,
    const std::shared_ptr<Container>& dest
) {
    auto parent = node->getParent();
    if (auto container = dynamic_cast<Container*>(parent)) {
        container->remove(node.get());
    }
    if (parent) {
        parent->scrolled(0);
    }
    dest->add(node);
}

vec2supplier UINode::getPositionFunc() const {
    return positionfunc;
}

void UINode::setPositionFunc(vec2supplier func) {
    positionfunc = std::move(func);
}

vec2supplier UINode::getSizeFunc() const {
    return sizefunc;
}

void UINode::setSizeFunc(vec2supplier func) {
    sizefunc = std::move(func);
}

void UINode::setId(const std::string& id) {
    this->id = id;
}

const std::string& UINode::getId() const {
    return id;
}

void UINode::reposition() {
    if (sizefunc) {
        glm::ivec2 newSize = sizefunc();
        glm::ivec2 defsize = newSize;
        if (parent) {
            defsize = parent->getSize();
        }
        newSize.x = newSize.x == 0 ? size.x : newSize.x;
        newSize.y = newSize.y == 0 ? size.y : newSize.y;
        setSize(
            {newSize.x < 0 ? defsize.x + (newSize.x + 1) : newSize.x,
             newSize.y < 0 ? defsize.y + (newSize.y + 1) : newSize.y}
        );
    }
    if (positionfunc) {
        setPos(positionfunc());
    }
}

void UINode::setGravity(Gravity gravity) {
    if (gravity == Gravity::NONE) {
        setPositionFunc(nullptr);
        return;
    }
    setPositionFunc([this, gravity](){
        auto parent = getParent();
        if (parent == nullptr) {
            return getPos();
        }
        glm::vec4 margin = getMargin();
        glm::vec2 size = getSize();
        glm::vec2 parentSize = parent->getSize();

        float x = 0.0f, y = 0.0f;
        switch (gravity) {
            case Gravity::TOP_LEFT:
            case Gravity::CENTER_LEFT:
            case Gravity::BOTTOM_LEFT: x = margin.x; break;
            case Gravity::TOP_CENTER:
            case Gravity::CENTER_CENTER:
            case Gravity::BOTTOM_CENTER: x = (parentSize.x-size.x)/2.0f; break;
            case Gravity::TOP_RIGHT:
            case Gravity::CENTER_RIGHT:
            case Gravity::BOTTOM_RIGHT: x = parentSize.x-size.x-margin.z; break;
            default: break;
        }
        switch (gravity) {
            case Gravity::TOP_LEFT:
            case Gravity::TOP_CENTER:
            case Gravity::TOP_RIGHT: y = margin.y; break;
            case Gravity::CENTER_LEFT:
            case Gravity::CENTER_CENTER:
            case Gravity::CENTER_RIGHT: y = (parentSize.y-size.y)/2.0f; break;
            case Gravity::BOTTOM_LEFT:
            case Gravity::BOTTOM_CENTER:
            case Gravity::BOTTOM_RIGHT: y = parentSize.y-size.y-margin.w; break;
            default: break;
        }
        return glm::vec2(x, y);
    });

    if (parent) {
        reposition();
    }
}

bool UINode::isSubnodeOf(const UINode* node) {
    if (parent == nullptr) {
        return false;
    }
    if (parent == node) {
        return true;
    }
    return parent->isSubnodeOf(node);
}

void UINode::getIndices(
    const std::shared_ptr<UINode>& node,
    std::unordered_map<std::string, std::shared_ptr<UINode>>& map
) {
    const std::string& id = node->getId();
    if (!id.empty()) {
        map[id] = node;
    }
    auto container = std::dynamic_pointer_cast<gui::Container>(node);
    if (container) {
        for (const auto& subnode : container->getNodes()) {
            getIndices(subnode, map);
        }
    }
}
std::shared_ptr<UINode> UINode::find(
    const std::shared_ptr<UINode>& node,
    const std::string& id
) {
    if (node->getId() == id) {
        return node;
    }
    if (auto container = std::dynamic_pointer_cast<Container>(node)) {
        for (const auto& subnode : container->getNodes()) {
            if (auto found = UINode::find(subnode, id)) {
                return found;
            }
        }
    }
    return nullptr;
}
