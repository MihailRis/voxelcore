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

UINode* UINode::listenAction(const onaction& action) {
    actions.listen(action);
    return this;
}

UINode* UINode::listenDoubleClick(const onaction& action) {
    doubleClickCallbacks.listen(action);
    return this;
}

UINode* UINode::listenFocus(const onaction& action) {
    focusCallbacks.listen(action);
    return this;
}

UINode* UINode::listenDefocus(const onaction& action) {
    defocusCallbacks.listen(action);
    return this;
}

void UINode::click(int, int) {
    pressed = true;
}

void UINode::doubleClick(int x, int y) {
    pressed = true;
    if (isInside(glm::vec2(x, y))) {
        doubleClickCallbacks.notify(gui);
    }
}

void UINode::mouseRelease(int x, int y) {
    pressed = false;
    if (isInside(glm::vec2(x, y))) {
        actions.notify(gui);
    }
}

bool UINode::isPressed() const {
    return pressed;
}

void UINode::onFocus() {
    focused = true;
    focusCallbacks.notify(gui);
}

void UINode::defocus() {
    focused = false;
    defocusCallbacks.notify(gui);
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

void UINode::setPos(glm::vec2 pos) {
    this->pos = pos;
}

glm::vec2 UINode::getPos() const {
    return pos;
}

glm::vec2 UINode::getSize() const {
    return size;
}

void UINode::setSize(glm::vec2 size) {
    this->size = glm::vec2(
        glm::max(minSize.x, glm::min(maxSize.x, size.x)),
        glm::max(minSize.y, glm::min(maxSize.y, size.y))
    );
}

glm::vec2 UINode::getMinSize() const {
    return minSize;
}

void UINode::setMinSize(glm::vec2 minSize) {
    this->minSize = minSize;
    setSize(getSize());
}

glm::vec2 UINode::getMaxSize() const {
    return maxSize;
}

void UINode::setMaxSize(glm::vec2 maxSize) {
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
    if (gravity == Gravity::none) {
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
            case Gravity::top_left:
            case Gravity::center_left:
            case Gravity::bottom_left: x = margin.x; break;
            case Gravity::top_center:
            case Gravity::center_center:
            case Gravity::bottom_center: x = (parentSize.x-size.x)/2.0f; break;
            case Gravity::top_right:
            case Gravity::center_right:
            case Gravity::bottom_right: x = parentSize.x-size.x-margin.z; break;
            default: break;
        }
        switch (gravity) {
            case Gravity::top_left:
            case Gravity::top_center:
            case Gravity::top_right: y = margin.y; break;
            case Gravity::center_left:
            case Gravity::center_center:
            case Gravity::center_right: y = (parentSize.y-size.y)/2.0f; break;
            case Gravity::bottom_left:
            case Gravity::bottom_center:
            case Gravity::bottom_right: y = parentSize.y-size.y-margin.w; break;
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

void UINode::setClassname(std::string _classname) {
    classname = _classname;
}

std::string UINode::getClassname() const {
    return classname;
}

void UINode::setName(std::string _name) {
    name = _name;
}

std::string UINode::getName() const {
    return name;
}

void UINode::applyStylesheet(const std::vector<StylesheetRule>& rules) {
    std::unordered_map<std::string, style::value> styles = calculateStylesheet(rules);
    for (const auto& [property, value] : styles) {
        switch (getStylePropertyType(property))
        {
        case StyleProperty::COLOR:
            setColor( value.asColor() );
            break;
        case StyleProperty::MARGIN:
            setMargin( value.asVec4() );
            break;
        case StyleProperty::Z_INDEX:
            setZIndex( value.asInt() );
            break;
        default:
            break;
        }
    }
}

std::unordered_map<std::string, style::value> UINode::calculateStylesheet(const std::vector<StylesheetRule>& rules) {
    std::vector<AppliedRule> appliedRules;
    appliedRules.reserve(rules.size());
    
    for (size_t i = 0; i < rules.size(); ++i) {
        const auto& rule = rules[i];
        
        // Разбиваем составные селекторы (button, .menu)
        std::vector<std::string> selectors;
        size_t start = 0;
        size_t comma;
        do {
            comma = rule.selector.find(',', start);
            size_t end = (comma == std::string::npos) ? rule.selector.length() : comma;
            std::string sel = trim(rule.selector.substr(start, end - start));
            if (!sel.empty()) {
                selectors.push_back(sel);
            }
            start = comma + 1;
        } while (comma != std::string::npos);
        
        bool selectorMatches = false;
        
        for (const auto& selector : selectors) {
            // Проверяем, является ли селектор универсальным *
            if (selector == "*") {
                // Универсальный селектор применяется ко всем элементам
                selectorMatches = true;
                
                // Специфичность для * = 0
                appliedRules.push_back({&rule, 0, i});
                break; // Другие селекторы в этом правиле не проверяем
            }
            
            // Парсим обычный селектор
            std::string tag, id, classname;
            std::vector<std::string> pseudoClasses;
            
            size_t pos = 0;
            while (pos < selector.length()) {
                if (std::isspace(selector[pos])) {
                    pos++;
                    continue;
                }
                
                if (selector[pos] == '#') {
                    pos++;
                    size_t end = selector.find_first_of(".:#[] ", pos);
                    if (end == std::string::npos) end = selector.length();
                    id = selector.substr(pos, end - pos);
                    pos = end;
                } 
                else if (selector[pos] == '.') {
                    pos++;
                    size_t end = selector.find_first_of(".:#[] ", pos);
                    if (end == std::string::npos) end = selector.length();
                    classname = selector.substr(pos, end - pos);
                    pos = end;
                } 
                else if (selector[pos] == ':') {
                    pos++;
                    size_t end = selector.find_first_of(".:#[] ", pos);
                    if (end == std::string::npos) end = selector.length();
                    pseudoClasses.push_back(selector.substr(pos, end - pos));
                    pos = end;
                } 
                else {
                    size_t end = selector.find_first_of(".:#[] ", pos);
                    if (end == std::string::npos) end = selector.length();
                    tag = selector.substr(pos, end - pos);
                    pos = end;
                }
            }
            
            // Проверяем соответствие узлу
            bool matches = true;
            
            // Проверяем тег (если указан и не *)
            if (!tag.empty() && tag != "*" && this->getName() != tag) {
                matches = false;
            }
            
            // Проверяем ID (если указан)
            if (!id.empty() && this->getId() != id) {
                matches = false;
            }
            
            // Проверяем класс (если указан)
            if (!classname.empty() && this->getClassname() != classname) {
                matches = false;
            }
            
            // Проверяем все псевдоклассы
            for (const auto& pseudo : pseudoClasses) {
                if (pseudo == "hover" && !this->isHover()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "active" && !this->isPressed()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "focus" && !this->isFocused()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "disabled" && this->isEnabled()) {
                    matches = false;
                    break;
                }
            }
            
            if (matches) {
                selectorMatches = true;
                
                // Вычисляем специфичность селектора
                int specificity = 0;
                
                if (!id.empty()) specificity += 100;
                if (!classname.empty()) specificity += 10;
                if (!tag.empty() && tag != "*") specificity += 1;
                specificity += pseudoClasses.size() * 10;
                
                appliedRules.push_back({&rule, specificity, i});
                break; // Если один из составных селекторов подходит, остальные не проверяем
            }
        }
    }
    
    // Сортируем правила по специфичности
    std::sort(appliedRules.begin(), appliedRules.end(), 
        [](const AppliedRule& a, const AppliedRule& b) {
            if (a.specificity != b.specificity) {
                return a.specificity > b.specificity;
            }
            return a.index < b.index; // Ранние правила имеют приоритет при одинаковой специфичности
        });
    
    // Применяем стили
    std::unordered_map<std::string, style::value> styles;
    
    for (const auto& applied : appliedRules) {
        for (const auto& [property, value] : applied.rule->declarations) {
            // Более специфичные правила переопределяют менее специфичные
            if (styles.find(property) == styles.end()) {
                styles[property] = value;
            }
        }
    }
    
    return styles;
}