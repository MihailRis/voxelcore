#include "StylesheetParser.hpp"
#include <cctype>
#include <sstream>
#include <variant>

// Утилита: trim
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\n\r");
    size_t end = s.find_last_not_of(" \t\n\r");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

// Разделение составных селекторов: `button, .menu`
static std::vector<std::string> splitSelectors(const std::string& sel) {
    std::vector<std::string> result;
    std::stringstream ss(sel);
    std::string part;
    while (std::getline(ss, part, ',')) {
        auto t = trim(part);
        if (!t.empty()) result.push_back(t);
    }
    return result;
}

std::vector<StylesheetRule> StylesheetParser::parse(const std::string& source) {
    std::vector<StylesheetRule> rules;
    size_t pos = 0;
    const size_t len = source.length();

    while (pos < len) {
        if (std::isspace(source[pos])) {
            ++pos;
            continue;
        }
        if (pos + 1 < len && source.substr(pos, 2) == "/*") {
            auto end = source.find("*/", pos);
            if (end == std::string::npos) break;
            pos = end + 2;
            continue;
        }

        auto open_brace = source.find('{', pos);
        if (open_brace == std::string::npos) break;

        auto selector_str = trim(source.substr(pos, open_brace - pos));
        pos = open_brace + 1;

        auto close_brace = source.find('}', pos);
        if (close_brace == std::string::npos) break;

        auto body = source.substr(pos, close_brace - pos);
        pos = close_brace + 1;

        std::unordered_map<std::string, style::value> decl;
        std::stringstream sb(body);
        std::string line;
        while (std::getline(sb, line, ';')) {
            auto colon = line.find(':');
            if (colon != std::string::npos) {
                std::string prop = trim(line.substr(0, colon));
                std::string value_str = trim(line.substr(colon + 1));
                if (!prop.empty() && !value_str.empty()) {
                    // Автоматическое определение типа
                    style::value parsed_value = value_str;

                    decl[prop] = parsed_value;
                }
            }
        }

        auto selectors = splitSelectors(selector_str);
        for (const auto& sel : selectors) {
            rules.push_back({ sel, decl });
        }
    }

    return rules;
}
