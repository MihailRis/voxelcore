#pragma once

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <optional>
#include <algorithm>

#include "SelectorParser.hpp"
#include "DeclarationParser.hpp"
#include "coders/BasicParser.hpp"
#include "stylesheet.h"

class StylesheetParser : public BasicParser<char> {
public:
    StylesheetParser(std::string_view file, std::string_view source);

    style::Stylesheet parse();
private:
    std::optional<style::Rule> parseRule();
    std::vector<style::Selector> parseSelectorList();
    std::vector<style::Declaration> parseDeclarations();
    std::optional<style::Declaration> parseDeclaration();
    std::string parseCSSIdentifier();

    bool is_css_identifier_start(char c) const;
    bool is_css_identifier_part(char c) const;

};
