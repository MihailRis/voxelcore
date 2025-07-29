#include "Container.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

#include "../style/StylesheetParser.h"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"

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
    int diff = (actualLength - size.y);
    if (scrollable && diff > 0 &&
        pos.x > calcPos().x + getSize().x - scrollBarWidth) {
        return UINode::getAt(pos);
    }

    for (int i = nodes.size() - 1; i >= 0; i--) {
        auto& node = nodes[i];
        if (!node->isVisible()) continue;
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
    int diff = (actualLength - size.y);
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
    intervalEvents.erase(
        std::remove_if(
            intervalEvents.begin(),
            intervalEvents.end(),
            [](const IntervalEvent& event) { return event.repeat == 0; }
        ),
        intervalEvents.end()
    );
}

void Container::scrolled(int value) {
    int diff = (actualLength - getSize().y);
    if (scroll < 0 && diff <= 0) {
        scroll = 0;
    }
    if (diff > 0 && scrollable) {
        scroll += value * scrollStep;
        if (scroll > 0) scroll = 0;
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

static std::string trim(const std::string& s) {
    if (s.empty()) return s;

    size_t start = s.find_first_not_of(" \t\n\r");
    size_t end = s.find_last_not_of(" \t\n\r");

    if (start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

std::unordered_map<std::string, style::value> calculateStylesheet(
    const UINode& node, 
    const std::vector<StylesheetRule>& rules
) {
    struct AppliedRule {
        const StylesheetRule* rule;
        int specificity;
        size_t index;
    };
    
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
            if (!tag.empty() && tag != "*" && node.getName() != tag) {
                matches = false;
            }
            
            // Проверяем ID (если указан)
            if (!id.empty() && node.getId() != id) {
                matches = false;
            }
            
            // Проверяем класс (если указан)
            if (!classname.empty() && node.getClassname() != classname) {
                matches = false;
            }
            
            // Проверяем все псевдоклассы
            for (const auto& pseudo : pseudoClasses) {
                if (pseudo == "hover" && !node.isHover()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "active" && !node.isPressed()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "focus" && !node.isFocused()) {
                    matches = false;
                    break;
                } 
                else if (pseudo == "disabled" && node.isEnabled()) {
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

std::string stylesheet_src = R"(
    button:hover {
        color: #575757ff;
    }
    button { 
        color: #000000;
    }
    * {
        color: #00ff00;
    }
)";

void Container::draw(const DrawContext& pctx, const Assets& assets) {
    glm::vec2 pos = calcPos();
    glm::vec2 size = getSize();
    drawBackground(pctx, assets);

    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);
    if (!nodes.empty()) {
        batch->flush();
        DrawContext ctx = pctx.sub();
        ctx.setScissors(
            glm::vec4(pos.x, pos.y, glm::ceil(size.x), glm::ceil(size.y))
        );
        for (const auto& node : nodes) {
            if (node->isVisible()) {
                if (node->getName() != "") {
                    std::vector<StylesheetRule> styles = StylesheetParser::parse(stylesheet_src);
                    std::unordered_map<std::string, style::value> node_styles = calculateStylesheet(*node, styles);
                    for (const auto& [property, value] : node_styles) {
                        switch (getStylePropertyType(property))
                        {
                        case StyleProperty::COLOR:
                            node->setColor( value.asColor() );
                            break;
                        case StyleProperty::MARGIN:
                            node->setMargin( value.asVec4() );
                            break;

                        default:
                            break;
                        }
                    }
                }

                node->draw(pctx, assets);
            }
        }

        int diff = (actualLength - size.y);
        if (scrollable && diff > 0) {
            int h =
                glm::max(size.y / actualLength * size.y, scrollBarWidth / 2.0f);
            batch->untexture();
            batch->setColor(glm::vec4(1, 1, 1, 0.3f));
            batch->rect(
                pos.x + size.x - scrollBarWidth,
                pos.y - scroll / static_cast<float>(diff) * (size.y - h),
                scrollBarWidth,
                h
            );
        }
        batch->flush();
    }
}

void Container::drawBackground(const DrawContext& pctx, const Assets&) {
    glm::vec4 color = calcColor();
    if (color.a <= 0.001f) return;
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
}

void Container::add(const std::shared_ptr<UINode>& node, glm::vec2 pos) {
    node->setPos(pos);
    add(node);
}

void Container::remove(UINode* selected) {
    selected->setParent(nullptr);
    nodes.erase(
        std::remove_if(
            nodes.begin(),
            nodes.end(),
            [selected](const std::shared_ptr<UINode>& node) {
                return node.get() == selected;
            }
        ),
        nodes.end()
    );
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

void Container::listenInterval(float interval, ontimeout callback, int repeat) {
    intervalEvents.push_back({std::move(callback), interval, 0.0f, repeat});
}

void Container::setSize(glm::vec2 size) {
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
    std::stable_sort(
        nodes.begin(), nodes.end(), [](const auto& a, const auto& b) {
            return a->getZIndex() < b->getZIndex();
        }
    );
}

void Container::setScroll(int scroll) {
    this->scroll = scroll;
}

const std::vector<std::shared_ptr<UINode>>& Container::getNodes() const {
    return nodes;
}
