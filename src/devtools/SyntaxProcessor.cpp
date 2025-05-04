#include "SyntaxProcessor.hpp"

#include "coders/commons.hpp"
#include "coders/syntax_parser.hpp"
#include "graphics/core/Font.hpp"

using namespace devtools;

static std::unique_ptr<FontStylesScheme> build_styles(
    const std::vector<devtools::Token>& tokens
) {
    using devtools::TokenTag;
    FontStylesScheme styles {
        {
            {false, false, false, false, glm::vec4(0.8f, 0.8f, 0.8f, 1)}, // default
            {true, false, false, false, glm::vec4(0.9, 0.6f, 0.4f, 1)},   // keyword
            {false, false, false, false, glm::vec4(0.4, 0.8f, 0.5f, 1)},  // string
            {false, false, false, false, glm::vec4(0.3, 0.3f, 0.3f, 1)},  // comment
            {true, false, false, false, glm::vec4(1.0f, 0.2f, 0.1f, 1)},  // unexpected
        }, 
        {}
    };
    size_t offset = 0;
    for (int i = 0; i < tokens.size(); i++) {
        const auto& token = tokens.at(i);
        if (token.tag != TokenTag::KEYWORD &&
            token.tag != TokenTag::STRING &&
            token.tag != TokenTag::INTEGER &&
            token.tag != TokenTag::NUMBER &&
            token.tag != TokenTag::COMMENT &&
            token.tag != TokenTag::UNEXPECTED) {
            continue;
        }
        if (token.start.pos > offset) {
            styles.map.insert(styles.map.end(), token.start.pos - offset, 0);
        }
        offset = token.end.pos;
        int styleIndex;
        switch (token.tag) {
            case TokenTag::KEYWORD: styleIndex = SyntaxStyles::KEYWORD; break;
            case TokenTag::STRING:
            case TokenTag::INTEGER:
            case TokenTag::NUMBER: styleIndex = SyntaxStyles::LITERAL; break;
            case TokenTag::COMMENT: styleIndex = SyntaxStyles::COMMENT; break;
            case TokenTag::UNEXPECTED: styleIndex = SyntaxStyles::ERROR; break;
            default:
                styleIndex = 0;
                break;
        }
        styles.map.insert(
            styles.map.end(), token.end.pos - token.start.pos, styleIndex
        );
    }
    styles.map.push_back(0);
    return std::make_unique<FontStylesScheme>(std::move(styles));
}

void SyntaxProcessor::addSyntax(
    std::unique_ptr<Syntax> syntax
) {
    const auto ptr = syntax.get();
    langs.emplace_back(std::move(syntax));

    for (auto& ext : ptr->extensions) {
        langsExtensions[ext] = ptr;
    }
}

std::unique_ptr<FontStylesScheme> SyntaxProcessor::highlight(
    const std::string& ext, std::wstring_view source
) const {
    const auto& found = langsExtensions.find(ext);
    if (found == langsExtensions.end()) {
        return nullptr;
    } 
    const auto& syntax = *found->second;
    try {
        auto tokens = tokenize(syntax, "<string>", source);
        return build_styles(tokens);
    } catch (const parsing_error& err) {
        return nullptr;
    }
}
