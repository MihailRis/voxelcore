#include "value.h"
#include <algorithm>
#include <cctype>

namespace style {

// Убираем пробелы
static std::string trim(const std::string& s) {
    auto wsfront = std::find_if(s.begin(), s.end(), [](int c){return !std::isspace(c);});
    auto wsback = std::find_if(s.rbegin(), s.rend(), [](int c){return !std::isspace(c);}).base();
    return (wsback <= wsfront ? std::string() : std::string(wsfront, wsback));
}

// Проверка, что строка — число
static bool is_number(const std::string& s) {
    if (s.empty()) return false;
    size_t i = s[0] == '-' ? 1 : 0;
    return i < s.size() && std::all_of(s.begin() + i, s.end(), ::isdigit);
}

// Парсинг цвета: #RGB, #RGBA, #RRGGBB, #RRGGBBAA, rgba()
static std::optional<glm::vec4> parse_color(const std::string& str) {
    auto s = trim(str);
    if (s.empty()) return std::nullopt;

    if (s[0] == '#') {
        std::string hex = s.substr(1);
        if (hex.length() == 3) {
            int r = (hex[0] - '0') * 16 + (hex[0] - '0');
            int g = (hex[1] - '0') * 16 + (hex[1] - '0');
            int b = (hex[2] - '0') * 16 + (hex[2] - '0');
            return glm::vec4(r/255.0f, g/255.0f, b/255.0f, 1.0f);
        } else if (hex.length() == 4) {
            int r = (hex[0] - '0') * 16 + (hex[0] - '0');
            int g = (hex[1] - '0') * 16 + (hex[1] - '0');
            int b = (hex[2] - '0') * 16 + (hex[2] - '0');
            int a = (hex[3] - '0') * 16 + (hex[3] - '0');
            return glm::vec4(r/255.0f, g/255.0f, b/255.0f, a/255.0f);
        } else if (hex.length() == 6) {
            int r = std::stoi(hex.substr(0, 2), nullptr, 16);
            int g = std::stoi(hex.substr(2, 2), nullptr, 16);
            int b = std::stoi(hex.substr(4, 2), nullptr, 16);
            return glm::vec4(r/255.0f, g/255.0f, b/255.0f, 1.0f);
        } else if (hex.length() == 8) {
            int r = std::stoi(hex.substr(0, 2), nullptr, 16);
            int g = std::stoi(hex.substr(2, 2), nullptr, 16);
            int b = std::stoi(hex.substr(4, 2), nullptr, 16);
            int a = std::stoi(hex.substr(6, 2), nullptr, 16);
            return glm::vec4(r/255.0f, g/255.0f, b/255.0f, a/255.0f);
        }
    }

    if (s.rfind("rgba(", 0) == 0) {
        float r, g, b, a;
        if (std::sscanf(s.c_str(), "rgba(%f,%f,%f,%f)", &r, &g, &b, &a) == 4) {
            return glm::vec4(r/255.0f, g/255.0f, b/255.0f, a);
        }
    }

    // Именованные цвета (минимально)
    if (s == "red") return glm::vec4(1,0,0,1);
    if (s == "green") return glm::vec4(0,1,0,1);
    if (s == "blue") return glm::vec4(0,0,1,1);
    if (s == "white") return glm::vec4(1,1,1,1);
    if (s == "black") return glm::vec4(0,0,0,1);
    if (s == "transparent") return glm::vec4(0,0,0,0);

    return std::nullopt;
}

// === Реализация методов ===

int64_t style::value::asInt(int64_t def) const {
    if (std::holds_alternative<int64_t>(data)) {
        return std::get<int64_t>(data);
    }
    if (std::holds_alternative<double>(data)) {
        return static_cast<int64_t>(std::get<double>(data));
    }
    if (std::holds_alternative<bool>(data)) {
        return std::get<bool>(data) ? 1 : 0;
    }
    if (std::holds_alternative<std::string>(data)) {
        if (auto res = tryParse<int64_t>(std::get<std::string>(data))) {
            return *res;
        }
    }
    return def;
}

double style::value::asFloat(double def) const {
    if (std::holds_alternative<double>(data)) {
        return std::get<double>(data);
    }
    if (std::holds_alternative<int64_t>(data)) {
        return static_cast<double>(std::get<int64_t>(data));
    }
    if (std::holds_alternative<bool>(data)) {
        return std::get<bool>(data) ? 1.0 : 0.0;
    }
    if (std::holds_alternative<std::string>(data)) {
        if (auto res = tryParse<double>(std::get<std::string>(data))) {
            return *res;
        }
    }
    return def;
}

float style::value::asFloat(float def) const {
    return static_cast<float>(asFloat(static_cast<double>(def)));
}

bool style::value::asBool(bool def) const {
    if (std::holds_alternative<bool>(data)) {
        return std::get<bool>(data);
    }
    if (std::holds_alternative<int64_t>(data)) {
        return std::get<int64_t>(data) != 0;
    }
    if (std::holds_alternative<double>(data)) {
        return std::get<double>(data) != 0.0;
    }
    if (std::holds_alternative<std::string>(data)) {
        auto& s = std::get<std::string>(data);
        return s == "true" || s == "1" || !s.empty();
    }
    return def;
}

std::string style::value::asString(const std::string& def) const {
    if (std::holds_alternative<std::string>(data)) {
        return std::get<std::string>(data);
    }
    if (std::holds_alternative<int64_t>(data)) {
        return std::to_string(std::get<int64_t>(data));
    }
    if (std::holds_alternative<double>(data)) {
        return std::to_string(std::get<double>(data));
    }
    if (std::holds_alternative<bool>(data)) {
        return std::get<bool>(data) ? "true" : "false";
    }
    if (std::holds_alternative<glm::vec4>(data)) {
        auto v = std::get<glm::vec4>(data);
        return "vec4(" + std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z) + "," + std::to_string(v.w) + ")";
    }
    if (std::holds_alternative<glm::vec3>(data)) {
        auto v = std::get<glm::vec3>(data);
        return "vec3(" + std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z) + ")";
    }
    if (std::holds_alternative<glm::vec2>(data)) {
        auto v = std::get<glm::vec2>(data);
        return "vec2(" + std::to_string(v.x) + "," + std::to_string(v.y) + ")";
    }
    return def;
}

glm::vec2 style::value::asVec2(const glm::vec2& def) const {
    if (std::holds_alternative<glm::vec2>(data)) {
        return std::get<glm::vec2>(data);
    }
    if (std::holds_alternative<std::string>(data)) {
        std::string s = std::get<std::string>(data);
        float x, y;
        if (std::sscanf(s.c_str(), "%f,%f", &x, &y) == 2) {
            return glm::vec2(x, y);
        }
    }
    return def;
}

glm::vec3 style::value::asVec3(const glm::vec3& def) const {
    if (std::holds_alternative<glm::vec3>(data)) {
        return std::get<glm::vec3>(data);
    }
    if (std::holds_alternative<std::string>(data)) {
        std::string s = std::get<std::string>(data);
        float x, y, z;
        if (std::sscanf(s.c_str(), "%f,%f,%f", &x, &y, &z) == 3) {
            return glm::vec3(x, y, z);
        }
    }
    return def;
}

glm::vec4 style::value::asVec4(const glm::vec4& def) const {
    if (std::holds_alternative<glm::vec4>(data)) {
        return std::get<glm::vec4>(data);
    }
    if (std::holds_alternative<std::string>(data)) {
        std::string s = std::get<std::string>(data);
        float x, y, z, w;
        if (std::sscanf(s.c_str(), "%f,%f,%f,%f", &x, &y, &z, &w) == 4) {
            return glm::vec4(x, y, z, w);
        }
    }
    return def;
}

glm::vec4 style::value::asColor(const glm::vec4& def) const {
    if (std::holds_alternative<glm::vec4>(data)) {
        auto v = std::get<glm::vec4>(data);
        return glm::vec4(v.x/255.0f, v.y/255.0f, v.z/255.0f, v.w/255.0f);
    }
    if (std::holds_alternative<std::string>(data)) {
        if (auto col = parse_color(std::get<std::string>(data))) {
            return *col;
        }
    }
    return def;
}

std::string style::value::type_name() const {
    if (std::holds_alternative<std::monostate>(data)) return "null";
    if (std::holds_alternative<std::string>(data)) return "string";
    if (std::holds_alternative<int64_t>(data)) return "int";
    if (std::holds_alternative<double>(data)) return "float";
    if (std::holds_alternative<bool>(data)) return "bool";
    if (std::holds_alternative<glm::vec2>(data)) return "vec2";
    if (std::holds_alternative<glm::vec3>(data)) return "vec3";
    if (std::holds_alternative<glm::vec4>(data)) return "vec4";
    return "unknown";
}

std::string style::value::toString() const {
    if (std::holds_alternative<std::string>(data)) return std::get<std::string>(data);
    if (std::holds_alternative<int64_t>(data)) return std::to_string(std::get<int64_t>(data));
    if (std::holds_alternative<double>(data)) return std::to_string(std::get<double>(data));
    if (std::holds_alternative<bool>(data)) return std::get<bool>(data) ? "true" : "false";
    if (std::holds_alternative<glm::vec2>(data)) {
        auto v = std::get<glm::vec2>(data);
        return std::to_string(v.x) + "," + std::to_string(v.y);
    }
    if (std::holds_alternative<glm::vec3>(data)) {
        auto v = std::get<glm::vec3>(data);
        return std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z);
    }
    if (std::holds_alternative<glm::vec4>(data)) {
        auto v = std::get<glm::vec4>(data);
        return std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z) + "," + std::to_string(v.w);
    }
    return "null";
}

}