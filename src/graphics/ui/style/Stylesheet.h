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
}