#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Stylesheet.hpp"
#include "value.h"

struct AppliedRule {
    const StylesheetRule* rule;
    int specificity;
    size_t index;
};

std::string trim(const std::string& s);

class StylesheetParser {
public:
    static std::vector<StylesheetRule> parse(const std::string& source);
};