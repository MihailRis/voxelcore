#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "Stylesheet.h"

namespace style {

/**
 * Парсер CSS-подобных стилей
 */
class StylesheetParser {
public:
    struct ParseResult {
        bool success = false;
        std::string error_message;
        std::vector<StyleRule> rules;
    };

private:
    std::string source_;
    size_t position_ = 0;
    size_t line_ = 1;
    size_t column_ = 1;

public:
    StylesheetParser() = default;
    
    
    ParseResult parse(const std::string& css_source);
    
    
    std::vector<Selector> parseSelector(const std::string& selector_string);
    std::unordered_map<PropertyID, value> parseDeclarations(const std::string& declarations_string);
    
    
    static std::vector<std::string> splitSelectors(const std::string& selector_string);
    static std::pair<std::string, std::string> splitDeclaration(const std::string& declaration);
    
private:
    
    void skipUntil(char delimiter);
    void skipWhitespace();
    void skipComments();
    char peek() const;
    char peek(size_t offset) const;
    char consume();
    bool match(char c);
    bool match(const std::string& str);
    std::string consumeWhile(bool (*predicate)(char));
    std::string consumeIdentifier();
    std::string consumeString();
    std::string consumeNumber();
    
    
    std::vector<Selector> parseSelectorGroup();
    Selector parseSimpleSelector();
    std::unordered_map<PropertyID, value> parseDeclarationBlock();
    std::pair<PropertyID, value> parseDeclaration();
    
    
    ParseResult makeError(const std::string& message) const;
    std::string getCurrentPosition() const;
    
    
    static bool isWhitespace(char c);
    static bool isIdentifierStart(char c);
    static bool isIdentifierChar(char c);
    static bool isNumberChar(char c);
};


namespace parser {
    StylesheetParser::ParseResult parseCSS(const std::string& css_string);
    
    
    bool parseAndAddToStylesheet(Stylesheet& stylesheet, const std::string& css_string);
}

} 