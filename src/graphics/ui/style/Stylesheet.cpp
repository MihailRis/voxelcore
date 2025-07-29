#include <cctype>
#include <sstream>
#include <variant>

#include "Stylesheet.hpp"

StyleProperty getStylePropertyType(const std::string& property) {
    if (property == "color") return StyleProperty::COLOR;
    if (property == "margin") return StyleProperty::MARGIN;
    if (property == "z-index") return StyleProperty::Z_INDEX;
    return StyleProperty::UNKNOWN;
}