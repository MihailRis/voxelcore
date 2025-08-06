#include "SelectorParser.hpp"
using namespace style;

Selector CSSSelectorParser::parse() {
    try {
        skipWhitespace();

        std::vector<SimpleSelector> first_selectors =
            parseSimpleSelectorSequence();

        if (!hasNext()) {
            if (first_selectors.size() == 1) {
                return Selector(first_selectors.front());
            } else {
                return Selector(ComplexSelector(first_selectors));
            }
        }

        ComplexSelector complex_selector;
        complex_selector.first = std::move(first_selectors);

        while (hasNext()) {
            Combinator combinator = parseCombinator();

            skipWhitespace();
            if (!hasNext()) {
                break;
            }

            std::vector<SimpleSelector> next_selectors =
                parseSimpleSelectorSequence();

            if (next_selectors.empty()) {
                break;
            }

            complex_selector.parts.emplace_back(
                combinator, std::move(next_selectors)
            );
        }

        return Selector(std::move(complex_selector));
    } catch (const parsing_error& e) {
        std::cerr << "Selector parsing error: " << e.what() << "\n";
        throw;
    }
}

std::vector<SimpleSelector> CSSSelectorParser::parseSimpleSelectorSequence() {
    std::vector<SimpleSelector> selectors;
    skipWhitespace();
    bool has_pseudo_prefix = false;

    while (hasNext()) {
        char next = peekNoJump();
        if (next == ',' || next == '{' || next == '}' || next == ';' ||
            is_combinator(next)) {
            break;
        }

        if (next == ':') {
            has_pseudo_prefix = true;
            SimpleSelector selector;
            selector.type = SimpleSelectorType::Tag;
            selector.value = "";
            selectors.push_back(
                std::move(selector)
            );
            break;
        } else {
            SimpleSelector selector = parseSingleSimpleSelector();
            selectors.push_back(std::move(selector));
        }
        skipWhitespace();
    }
    parsePseudoClassesForSequence(selectors, has_pseudo_prefix);
    return selectors;
}

SimpleSelector CSSSelectorParser::parseSingleSimpleSelector() {
    char c = peekNoJump();
    if (c == '*') {
        nextChar();
        return SimpleSelector(SimpleSelectorType::Universal, "*");
        ;
    } else if (c == '#') {
        nextChar();
        return SimpleSelector(SimpleSelectorType::ID, parseIdentifier());
    } else if (c == '.') {
        nextChar();
        return SimpleSelector(SimpleSelectorType::Class, parseIdentifier());
        ;
    } else if (is_identifier_start(c)) {
        return SimpleSelector(SimpleSelectorType::Tag, parseIdentifier());
    } else {
        if (hasNext()) nextChar();
        return SimpleSelector(SimpleSelectorType::Tag, "");
    }
}

std::string CSSSelectorParser::parseIdentifier() {
    int start = pos;
    // Первый символ
    if (hasNext() && is_identifier_start(source[pos])) {
        pos++;
    } else {
        return "";
    }
    while (hasNext() && is_css_identifier_part(source[pos])) {
        pos++;
    }
    return std::string(source.substr(start, pos - start));
}

bool CSSSelectorParser::is_css_identifier_part(char c) const {

    return is_identifier_part(c) || c == '-';
}

Combinator CSSSelectorParser::parseCombinator() {
    skipWhitespace();
    if (!hasNext()) return Combinator::Descendant;

    char c = peekNoJump();
    switch (c) {
        case '>':
            nextChar();
            return Combinator::Child;
        case '+':
            nextChar();
            return Combinator::Adjacent;
        case '~':
            nextChar();
            return Combinator::GeneralSibling;
        default:
            return Combinator::Descendant;
    }
}

bool CSSSelectorParser::is_combinator(char c) const {
    return c == '>' || c == '+' || c == '~';
}

void CSSSelectorParser::parseAndAddPseudoClass(SimpleSelector& selector) {
    expect(':');
    std::string pseudo_name = parseIdentifier();

    std::string lower_name = pseudo_name;
    std::transform(
        lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower
    );

    PseudoClass pclass = PseudoClass::Custom;
    std::string argument = "";

    static const std::map<std::string, PseudoClass> pseudo_map = {
        {"hover", PseudoClass::Hover},
        {"focus", PseudoClass::Focus},
        {"active", PseudoClass::Active},
        {"checked", PseudoClass::Checked},
        {"disabled", PseudoClass::Disabled},
        {"nth-child", PseudoClass::NthChild},
        {"nth-of-type", PseudoClass::NthOfType}
    };

    auto it = pseudo_map.find(lower_name);
    if (it != pseudo_map.end()) {
        pclass = it->second;
    } else {
        pclass = PseudoClass::Custom;
    }

    skipWhitespace();
    if (hasNext() && peekNoJump() == '(') {
        nextChar();
        int start = pos;
        int paren_count = 1;
        while (hasNext() && paren_count > 0) {
            char ch = source[pos];
            if (ch == '(') {
                paren_count++;
            } else if (ch == ')') {
                paren_count--;
            }
            if (paren_count > 0) {
                pos++;
            }
        }
        if (paren_count == 0) {
            argument = std::string(source.substr(start, pos - start));
            nextChar();
        } else {
            argument = std::string(source.substr(start));
        }
    }

    selector.pseudoClasses.emplace_back(pclass, argument);
}

void CSSSelectorParser::parsePseudoClassesForSequence(
    std::vector<style::SimpleSelector>& selectors, bool has_pseudo_prefix
) {
    if (has_pseudo_prefix) {
        parseAndAddPseudoClass(
            selectors.back()
        );
    }

    while (hasNext() && peekNoJump() == ':') {
        if (selectors.empty()) {
            selectors.emplace_back(SimpleSelectorType::Tag, "");
        }

        SimpleSelector temp_selector =
            selectors.front();
        temp_selector.pseudoClasses
            .clear();
        parseAndAddPseudoClass(temp_selector);

        for (auto& sel : selectors) {
            sel.pseudoClasses.insert(
                sel.pseudoClasses.end(),
                temp_selector.pseudoClasses.begin(),
                temp_selector.pseudoClasses.end()
            );
        }

        skipWhitespace();
    }
}

Selector parseCSSSelector(std::string_view selector_text) {
    CSSSelectorParser parser(selector_text);
    return parser.parse();
}