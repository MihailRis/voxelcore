#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "value.h"

namespace style {
    enum class PseudoClass {
        None,
        Hover,
        Focus,
        Active,
        Checked,
        Disabled,
        NthChild,  // :nth-child()
        NthOfType,
        Custom  // fallback
    };

    enum class SimpleSelectorType {
        Universal,  // *
        Tag,        // div, span
        ID,         // #id
        Class       // .class
    };

    struct SimpleSelector {
        SimpleSelectorType type;
        std::string value;  // tag, id or class
        std::vector<std::pair<PseudoClass, std::string>> pseudoClasses;

        SimpleSelector() = default;
        SimpleSelector(SimpleSelectorType t, std::string v) : type(t), value(std::move(v)) {}
        SimpleSelector(SimpleSelectorType t, std::string v, std::vector<std::pair<PseudoClass, std::string>> p) : type(t), value(std::move(v)), pseudoClasses(std::move(p)) {}
    };

    enum class Combinator {
        Descendant,     // child or inherit.
        Child,          // >
        Adjacent,       // +
        GeneralSibling  // ~
    };

    struct SelectorPart {
        Combinator combinator;
        std::vector<SimpleSelector> selectors;

        SelectorPart() = default;
        SelectorPart(Combinator comb, std::vector<SimpleSelector> sel)
            : combinator(comb), selectors(std::move(sel)) {
        }
    };

    struct ComplexSelector {
        std::vector<SimpleSelector> first;
        std::vector<SelectorPart> parts;

        ComplexSelector() = default;
        ComplexSelector(std::vector<SimpleSelector> f) : first(std::move(f)) {
        }
        ComplexSelector(
            std::vector<SimpleSelector> f, std::vector<SelectorPart> p
        )
            : first(std::move(f)), parts(std::move(p)) {
        }
    };

    struct Selector {
        std::variant<SimpleSelector, ComplexSelector> data;

        Selector() : data(SimpleSelector {}) {
        }
        Selector(SimpleSelector simple) : data(std::move(simple)) {
        }
        Selector(ComplexSelector complex) : data(std::move(complex)) {
        }
        Selector(SimpleSelectorType type, std::string value)
            : data(SimpleSelector(type, std::move(value))) {
        }
    };

    struct Declaration {
        std::string name;
        style::value value;

        Declaration() = default;
    };

    struct Rule {
        std::vector<Selector> selectors;
        std::vector<Declaration> declarations;

        Rule() = default;
    };

    struct Stylesheet {
        std::vector<Rule> rules;
    };

    struct ComputedStyle {
        std::unordered_map<std::string, style::value> properties;

        ComputedStyle() = default;

        explicit ComputedStyle(const std::vector<Declaration>& declarations) {
            for (const auto& decl : declarations) {
                properties[decl.name] = decl.value;
            }
        }

        const style::value* get(const std::string& property_name) const {
            auto it = properties.find(property_name);
            if (it != properties.end()) {
                return &it->second;
            }
            return nullptr;
        }

        style::value get(const std::string& property_name, style::value default_value) const {
             auto it = properties.find(property_name);
            if (it != properties.end()) {
                return it->second;
            }
            return default_value;
        }

        void set(const std::string& property_name, style::value value) {
            properties[std::move(property_name)] = std::move(value);
        }

        bool has(const std::string& property_name) const {
            return properties.find(property_name) != properties.end();
        }

        void remove(const std::string& property_name) {
            properties.erase(property_name);
        }

        size_t size() const {
            return properties.size();
        }

        bool empty() const {
            return properties.empty();
        }

        void clear() {
            properties.clear();
        }

        auto begin() { return properties.begin(); }
        auto end() { return properties.end(); }
        auto begin() const { return properties.begin(); }
        auto end() const { return properties.end(); }
        auto cbegin() const { return properties.cbegin(); }
        auto cend() const { return properties.cend(); }
    };
}