#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "value.h"

struct StylesheetRule {
    std::string selector;
    std::unordered_map<std::string, style::value> declarations;
};

enum class StyleProperty {
    COLOR,
    MARGIN,
    Z_INDEX,
    UNKNOWN
};

StyleProperty getStylePropertyType(const std::string& property);

class StylesheetParser {
public:
    static std::vector<StylesheetRule> parse(const std::string& source);
};