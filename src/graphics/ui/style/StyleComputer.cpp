#include "StyleComputer.hpp"

#include <algorithm>
#include <iostream>
#include <optional>
#include <type_traits>
#include <variant>
#include <queue>
#include <utility>

#include "DeclarationParser.hpp"

using namespace style;

void StyleComputer::set_stylesheets(const std::vector<Stylesheet>& sheets) {
    stylesheet.rules.clear();
    size_t total_rules = 0;
    for (const auto& sheet : sheets) {
        total_rules += sheet.rules.size();
    }
    stylesheet.rules.reserve(total_rules);

    for (const auto& sheet : sheets) {
        stylesheet.rules.insert(
            stylesheet.rules.end(), sheet.rules.begin(), sheet.rules.end()
        );
    }
}

void StyleComputer::compute(Node& node) {
    std::queue<std::pair<Node*, Node*>> queue;

    queue.push({&node, nullptr});

    while (!queue.empty()) {
        auto [current_node, parent_node] = queue.front();
        queue.pop();

        if (parent_node) {
            compute_base(*current_node, *parent_node);
        } else {
            compute_base(*current_node);
        }

        if (auto* element = std::get_if<Element>(&current_node->node_type)) {
            for (auto& child : current_node->children) {
                queue.push({&child, current_node});
            }
        }
    }
}

void StyleComputer::compute_base(Node& node, Node& parent) {
    if (auto* element = std::get_if<Element>(&node.node_type)) {
        if (auto* parent_element = std::get_if<Element>(&parent.node_type)) {
            element->style = parent_element->style;
            element->state_styles = parent_element->state_styles;
        }
    }

    compute_base(node);
}

void StyleComputer::compute_base(Node& node) {
    // Базовый стиль (без псевдоклассов)
    ComputedStyle base_style;

    // Мап для стилей псевдоклассов
    std::unordered_map<ElementState, ComputedStyle, ElementStateHash>
        state_styles;

    // Проходим по всем правилам
    for (auto& rule : stylesheet.rules) {
        for (auto& sel : rule.selectors) {
            if (match_selector(node, sel)) {
                // Проверяем, есть ли псевдоклассы в селекторе
                ElementState state = extract_element_state(sel);

                if (is_default_state(state)) {
                    // Базовый стиль (без псевдоклассов)
                    for (const auto& declaration : rule.declarations) {
                        base_style.set(declaration.name, declaration.value);
                    }
                } else {
                    // Стиль для конкретного состояния
                    auto& state_style = state_styles[state];
                    for (const auto& declaration : rule.declarations) {
                        state_style.set(declaration.name, declaration.value);
                    }
                }
            }
        }
    }

    // Применяем inline стили (они всегда к базовому состоянию)
    auto inline_style_attr = node.get_attribute("style");
    if (inline_style_attr && !inline_style_attr->empty()) {
        auto inline_declarations = parseInlineStyle(*inline_style_attr);
        for (const auto& decl : inline_declarations) {
            base_style.set(decl.name, decl.value);
        }
    }

    // Сохраняем стили в элемент
    if (auto* element = std::get_if<Element>(&node.node_type)) {
        element->style = std::move(base_style);
        element->state_styles = std::move(state_styles);
    }
}

// Проверяет, является ли состояние базовым (по умолчанию)
bool StyleComputer::is_default_state(const ElementState& state) {
    return !state.hover && !state.focus && !state.active && !state.checked &&
           !state.disabled && state.nth_child == 0 && state.nth_of_type == 0;
}

// Извлекает состояние элемента из селектора
ElementState StyleComputer::extract_element_state(const Selector& selector) {
    ElementState state;

    std::visit(
        [this, &state](const auto& sel) {
            using T = std::decay_t<decltype(sel)>;
            if constexpr (std::is_same_v<T, SimpleSelector>) {
                this->extract_state_from_simple_selector(sel, state);
            } else if constexpr (std::is_same_v<T, ComplexSelector>) {
                this->extract_state_from_complex_selector(sel, state);
            }
        },
        selector.data
    );

    return state;
}

void StyleComputer::extract_state_from_simple_selector(
    const SimpleSelector& selector, ElementState& state
) {
    for (const auto& [pseudo_class, value] : selector.pseudoClasses) {
        switch (pseudo_class) {
            case PseudoClass::Hover:
                state.hover = true;
                break;
            case PseudoClass::Focus:
                state.focus = true;
                break;
            case PseudoClass::Active:
                state.active = true;
                break;
            case PseudoClass::Checked:
                state.checked = true;
                break;
            case PseudoClass::Disabled:
                state.disabled = true;
                break;
            case PseudoClass::NthChild:
                // Парсим значение nth-child
                try {
                    state.nth_child = std::stoull(value);
                } catch (...) {
                    state.nth_child = 0;
                }
                break;
            case PseudoClass::NthOfType:
                // Парсим значение nth-of-type
                try {
                    state.nth_of_type = std::stoull(value);
                } catch (...) {
                    state.nth_of_type = 0;
                }
                break;
            default:
                break;
        }
    }
}

void StyleComputer::extract_state_from_complex_selector(
    const ComplexSelector& selector, ElementState& state
) {
    // Извлекаем состояние из first селекторов
    for (const auto& simple_sel : selector.first) {
        extract_state_from_simple_selector(simple_sel, state);
    }

    // Извлекаем состояние из всех частей
    for (const auto& part : selector.parts) {
        for (const auto& simple_sel : part.selectors) {
            extract_state_from_simple_selector(simple_sel, state);
        }
    }
}

bool StyleComputer::match_selector(
    Node& node, Selector& selector, Node* parent
) {
    return std::visit(
        [this, &node, parent](auto& sel) {
            return this->process_selector_variant(node, sel, parent);
        },
        selector.data
    );
}

template <typename T>
bool StyleComputer::process_selector_variant(
    Node& node, const T& selector, Node* parent
) {
    if constexpr (std::is_same_v<std::decay_t<T>, SimpleSelector>) {
        return process_simple_selector(node, selector);
    } else if constexpr (std::is_same_v<std::decay_t<T>, ComplexSelector>) {
        return process_complex_selector(node, selector, parent);
    }
}

bool StyleComputer::process_simple_selector(
    Node& node, const SimpleSelector& selector
) {
    if (auto* element = std::get_if<Element>(&node.node_type)) {
        bool basic_match = false;
        switch (selector.type) {
            case SimpleSelectorType::Universal:
                basic_match = true;
                break;

            case SimpleSelectorType::Tag:
                if (selector.value.empty()) {
                    basic_match = true;
                } else {
                    basic_match = element->data.tag_name == selector.value;
                }
                break;

            case SimpleSelectorType::ID: {
                auto element_id = element->data.getId();
                basic_match = element_id.has_value() &&
                              element_id.value() == selector.value;
                break;
            }

            case SimpleSelectorType::Class:
                basic_match = element->data.hasClass(selector.value);
                break;
        }

        if (basic_match) {
            return check_pseudo_classes(node, selector);
        }

        return false;
    }
    return false;
}

bool StyleComputer::process_complex_selector(
    Node& node, ComplexSelector selector, Node* parent
) {
    // Проверяем first селекторы на текущем узле
    for (const auto& simple_selector : selector.first) {
        if (!process_simple_selector(node, simple_selector)) {
            return false;
        }
    }

    // Если нет дополнительных частей, то успешно
    if (selector.parts.empty()) {
        return true;
    }

    // Пока реализуем только простые случаи с одной частью
    if (selector.parts.size() == 1) {
        const SelectorPart& part = selector.parts[0];
        Node* related_node = nullptr;

        // Определяем связанный узел в зависимости от комбинатора
        switch (part.combinator) {
            case Combinator::Child:
            case Combinator::Descendant:
                related_node = parent;  // Родитель передается как параметр
                break;

            case Combinator::Adjacent:
                // Для соседа нужно получить предыдущий sibling
                // Пока возвращаем nullptr, так как у нас нет доступа к siblings
                related_node = nullptr;
                break;

            case Combinator::GeneralSibling:
                // Для общего соседа тоже нужен доступ к siblings
                related_node = nullptr;
                break;
        }

        // Если связанный узел существует, проверяем селекторы в части
        if (related_node) {
            for (const auto& simple_selector : part.selectors) {
                if (!process_simple_selector(*related_node, simple_selector)) {
                    return false;
                }
            }
            return true;
        }
    }

    // Пока не поддерживаем более сложные случаи
    return selector.parts
        .empty();  // true только если нет дополнительных частей
}

bool StyleComputer::check_pseudo_classes(
    Node& node, const SimpleSelector& selector
) {
    if (selector.pseudoClasses.empty()) {
        return true;
    }

    if (auto* element = std::get_if<Element>(&node.node_type)) {
        for (const auto& [pseudo_class, value] : selector.pseudoClasses) {
            switch (pseudo_class) {
                case PseudoClass::Hover:
                    if (!element->state.hover) return false;
                    break;

                case PseudoClass::Focus:
                    if (!element->state.focus) return false;
                    break;

                case PseudoClass::Active:
                    if (!element->state.active) return false;
                    break;

                case PseudoClass::Checked:
                    if (!element->state.checked) return false;
                    break;

                case PseudoClass::Disabled:
                    if (!element->state.disabled) return false;
                    break;

                case PseudoClass::NthChild: {
                    if (!check_nth_child(*element, value)) return false;
                    break;
                }

                case PseudoClass::NthOfType: {
                    if (!check_nth_of_type(*element, value)) return false;
                    break;
                }

                case PseudoClass::Custom:
                    if (!check_custom_pseudo_class(*element, value))
                        return false;
                    break;

                case PseudoClass::None:
                    break;
            }
        }
        return true;
    }

    return false;
}

bool StyleComputer::check_nth_child(
    const Element& element, const std::string& formula
) {
    if (formula == "odd") {
        return element.state.nth_child % 2 == 1;
    } else if (formula == "even") {
        return element.state.nth_child % 2 == 0;
    } else {
        try {
            size_t pos;
            int n = std::stoi(formula, &pos);
            if (pos == formula.length()) {
                return static_cast<int>(element.state.nth_child) == n;
            }
        } catch (...) {
        }
    }
    return false;
}

bool StyleComputer::check_nth_of_type(
    const Element& element, const std::string& formula
) {
    if (formula == "odd") {
        return element.state.nth_of_type % 2 == 1;
    } else if (formula == "even") {
        return element.state.nth_of_type % 2 == 0;
    } else {
        try {
            size_t pos;
            int n = std::stoi(formula, &pos);
            if (pos == formula.length()) {
                return static_cast<int>(element.state.nth_of_type) == n;
            }
        } catch (...) {
        }
    }
    return false;
}

bool StyleComputer::check_custom_pseudo_class(
    const Element& element, const std::string& name
) {
    return false;
}