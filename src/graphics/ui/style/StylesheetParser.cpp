#include "StylesheetParser.h"
#include <sstream>
#include <algorithm>
#include <cctype>
#include <iostream>

namespace style {

StylesheetParser::ParseResult StylesheetParser::parse(const std::string& css_source) {
    source_ = css_source;
    position_ = 0;
    line_ = 1;
    column_ = 1;
    
    ParseResult result;
    result.rules.reserve(16); 
    
    try {
        while (position_ < source_.length()) {
            skipWhitespace();
            skipComments();
            
            if (position_ >= source_.length()) break;
            
            
            auto selectors = parseSelectorGroup();
            if (selectors.empty()) {
                return makeError("Expected selector");
            }
            
            skipWhitespace();
            
            
            if (!match('{')) {
                return makeError("Expected '{' after selector");
            }
            
            
            auto declarations = parseDeclarationBlock();
            
            
            if (!match('}')) {
                return makeError("Expected '}' after declarations");
            }
            
            position_++;
            
            
            if (!declarations.empty()) {
                result.rules.emplace_back(selectors, declarations);
            }
            
            skipWhitespace();
            skipComments();
        }
        
        result.success = true;
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "Exception during parsing: " + std::string(e.what());
    }
    
    return result;
}

std::vector<Selector> StylesheetParser::parseSelectorGroup() {
    std::vector<Selector> selectors;
    
    while (position_ < source_.length()) {
        skipWhitespace();
        
        if (position_ >= source_.length()) break;
        
        
        if (peek() == '{' || peek() == '}' || peek() == ';') {
            break;
        }
        
        
        Selector selector = parseSimpleSelector();
        
        selectors.push_back(selector);
        
        skipWhitespace();
        
        
        if (position_ >= source_.length() || source_[position_] != ',') {
            break;
        }
        
        consume(); 
        skipWhitespace();
    }
    
    return selectors;
}

Selector StylesheetParser::parseSimpleSelector() {
    if (position_ >= source_.length()) {
        return Selector(Selector::Type::Universal, "*");
    }
    
    char first_char = source_[position_];
    
    switch (first_char) {
        case '*':
            consume();
            return Selector(Selector::Type::Universal, "*");
            
        case '#': {
            consume(); 
            std::string id = consumeIdentifier();
            if (id.empty()) {
                return Selector(Selector::Type::Universal, "*"); 
            }
            return Selector(Selector::Type::ID, id);
        }
        
        case '.': {
            consume(); 
            std::string className = consumeIdentifier();
            if (className.empty()) {
                return Selector(Selector::Type::Universal, "*"); 
            }
            return Selector(Selector::Type::Class, className);
        }
        
        default: {
            if (isIdentifierStart(first_char)) {
                std::string tag = consumeIdentifier();
                return Selector(Selector::Type::Tag, tag);
            }
            
            return Selector(Selector::Type::Universal, "*");
        }
    }
}

std::unordered_map<PropertyID, value> StylesheetParser::parseDeclarationBlock() {
    std::unordered_map<PropertyID, value> declarations;
    
    while (position_ < source_.length() && peek() != '}') {
        skipWhitespace();
        
        if (position_ >= source_.length() || peek() == '}') break;
        
        
        auto declaration = parseDeclaration();
        if (declaration.first != PropertyID::Unknown) {
            declarations[declaration.first] = declaration.second;
        }
        
        skipWhitespace();
        
        
        if (position_ < source_.length() && peek() == ';') {
            consume();
        }
        
        skipWhitespace();
    }
    
    return declarations;
}

std::pair<PropertyID, value> StylesheetParser::parseDeclaration() {
    skipWhitespace();
    
    
    std::string property_name = consumeIdentifier();
    if (property_name.empty()) {
        return {PropertyID::Unknown, value()};
    }
    
    skipWhitespace();
    
    
    if (!match(':')) {
        skipUntil(';'); 
        return {PropertyID::Unknown, value()};
    }
    
    skipWhitespace();
    
    
    std::string value_str;
    while (position_ < source_.length() && 
           peek() != ';' && peek() != '}') {
        if (!isWhitespace(peek())) {
            value_str += consume();
        } else {
            
            value_str += consume();
        }
    }
    
    
    PropertyID prop_id = getPropertyID(property_name);
    if (prop_id == PropertyID::Unknown) {
        return {PropertyID::Unknown, value()};
    }
    
    
    value val = value::fromString(value_str);
    
    return {prop_id, val};
}


void StylesheetParser::skipUntil(char delimiter) {
    while (position_ < source_.length() && peek() != delimiter && peek() != '}') {
        consume();
    }
    if (position_ < source_.length() && peek() == delimiter) {
        consume();
    }
}


void StylesheetParser::skipWhitespace() {
    while (position_ < source_.length() && isWhitespace(peek())) {
        if (peek() == '\n') {
            line_++;
            column_ = 1;
        } else {
            column_++;
        }
        position_++;
    }
}

void StylesheetParser::skipComments() {
    while (position_ < source_.length() - 1) {
        if (source_[position_] == '/' && source_[position_ + 1] == '*') {
            
            position_ += 2;
            column_ += 2;
            
            while (position_ < source_.length() - 1) {
                if (source_[position_] == '*' && source_[position_ + 1] == '/') {
                    position_ += 2;
                    column_ += 2;
                    break;
                }
                if (source_[position_] == '\n') {  
                    line_++;
                    column_ = 1;
                } else {
                    column_++;
                }
                position_++;  
            }
        } else if (source_[position_] == '/' && source_[position_ + 1] == '/') {
            
            position_ += 2;
            column_ += 2;
            
            while (position_ < source_.length() && source_[position_] != '\n') {  
                position_++;
                column_++;
            }
            
            if (position_ < source_.length() && source_[position_] == '\n') {
                position_++;
                line_++;
                column_ = 1;
            }
        } else {
            break;
        }
    }
}

char StylesheetParser::peek() const {
    if (position_ >= source_.length()) return '\0';
    return source_[position_];
}

char StylesheetParser::peek(size_t offset) const {
    if (position_ + offset >= source_.length()) return '\0';
    return source_[position_ + offset];
}

char StylesheetParser::consume() {
    if (position_ >= source_.length()) return '\0';
    
    char c = source_[position_++];
    if (c == '\n') {
        line_++;
        column_ = 1;
    } else {
        column_++;
    }
    return c;
}

bool StylesheetParser::match(char c) {
    if (position_ < source_.length() && source_[position_] == c) {
        consume();
        return true;
    }
    return false;
}

bool StylesheetParser::match(const std::string& str) {
    if (position_ + str.length() <= source_.length()) {
        if (source_.substr(position_, str.length()) == str) {
            for (size_t i = 0; i < str.length(); ++i) {
                consume();
            }
            return true;
        }
    }
    return false;
}

std::string StylesheetParser::consumeWhile(bool (*predicate)(char)) {
    std::string result;
    while (position_ < source_.length() && predicate(peek())) {
        result += consume();
    }
    return result;
}

std::string StylesheetParser::consumeIdentifier() {
    return consumeWhile([](char c) {
        return isIdentifierChar(c);
    });
}

std::string StylesheetParser::consumeString() {
    if (position_ >= source_.length()) return "";
    
    char quote = peek();
    if (quote != '"' && quote != '\'') return "";
    
    consume(); 
    
    std::string result;
    while (position_ < source_.length() && peek() != quote) {
        if (peek() == '\\') {
            consume(); 
            if (position_ < source_.length()) {
                result += consume();
            }
        } else {
            result += consume();
        }
    }
    
    if (position_ < source_.length() && peek() == quote) {
        consume(); 
    }
    
    return result;
}

std::string StylesheetParser::consumeNumber() {
    return consumeWhile([](char c) {
        return isNumberChar(c);
    });
}


std::vector<std::string> StylesheetParser::splitSelectors(const std::string& selector_string) {
    std::vector<std::string> selectors;
    std::string current;
    int bracket_count = 0;
    
    for (char c : selector_string) {
        if (c == ',' && bracket_count == 0) {
            if (!current.empty()) {
                selectors.push_back(current);
                current.clear();
            }
        } else {
            if (c == '(') bracket_count++;
            else if (c == ')') bracket_count--;
            current += c;
        }
    }
    
    if (!current.empty()) {
        selectors.push_back(current);
    }
    
    return selectors;
}

std::pair<std::string, std::string> StylesheetParser::splitDeclaration(const std::string& declaration) {
    size_t colon_pos = declaration.find(':');
    if (colon_pos == std::string::npos) {
        return {"", ""};
    }
    
    std::string property = declaration.substr(0, colon_pos);
    std::string value = declaration.substr(colon_pos + 1);
    
    
    property.erase(0, property.find_first_not_of(" \t\r\n"));
    property.erase(property.find_last_not_of(" \t\r\n") + 1);
    
    value.erase(0, value.find_first_not_of(" \t\r\n"));
    value.erase(value.find_last_not_of(" \t\r\n") + 1);
    
    return {property, value};
}


bool StylesheetParser::isWhitespace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

bool StylesheetParser::isIdentifierStart(char c) {
    return std::isalpha(c) || c == '_' || c == '-';
}

bool StylesheetParser::isIdentifierChar(char c) {
    return std::isalnum(c) || c == '_' || c == '-';
}

bool StylesheetParser::isNumberChar(char c) {
    return std::isdigit(c) || c == '.' || c == '-' || c == '+';
}


StylesheetParser::ParseResult StylesheetParser::makeError(const std::string& message) const {
    ParseResult result;
    result.success = false;
    result.error_message = message + " at line " + std::to_string(line_) + 
                          ", column " + std::to_string(column_);
    return result;
}

std::string StylesheetParser::getCurrentPosition() const {
    return "line " + std::to_string(line_) + ", column " + std::to_string(column_);
}


namespace parser {
    StylesheetParser::ParseResult parseCSS(const std::string& css_string) {
        StylesheetParser parser;
        return parser.parse(css_string);
    }
    
    bool parseAndAddToStylesheet(Stylesheet& stylesheet, const std::string& css_string) {
        auto result = parseCSS(css_string);
        if (result.success) {
            for (const auto& rule : result.rules) {
                stylesheet.addRule(rule);
            }
            return true;
        }
        std::cerr << "CSS Parse Error: " << result.error_message << std::endl;
        return false;
    }
}


bool Stylesheet::parseAndAddRule(const std::string& css_string) {
    return parser::parseAndAddToStylesheet(*this, css_string);
}

} 