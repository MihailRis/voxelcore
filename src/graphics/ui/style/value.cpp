
#include "value.h"
#include <sstream>
#include <cctype>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace style {




value::value() = default; 


value::value(const value& other) : data(other.data) {
    
    
    if (other.color_cache) {
        color_cache = std::make_unique<glm::vec4>(*other.color_cache);
    }
    color_cache_valid = other.color_cache_valid;
    cached_string_value = other.cached_string_value;
    
}


value& value::operator=(const value& other) {
    
    if (this != &other) {
        data = other.data;
        if (other.color_cache) {
            color_cache = std::make_unique<glm::vec4>(*other.color_cache);
        } else {
            color_cache.reset();
        }
        color_cache_valid = other.color_cache_valid;
        cached_string_value = other.cached_string_value;
    }
    return *this;
}


value::value(value&& other) noexcept
    : data(std::move(other.data))
    , color_cache(std::move(other.color_cache))
    , color_cache_valid(other.color_cache_valid)
    , cached_string_value(std::move(other.cached_string_value)) {
    
    
}


value& value::operator=(value&& other) noexcept {
    
    if (this != &other) {
        data = std::move(other.data);
        color_cache = std::move(other.color_cache);
        color_cache_valid = other.color_cache_valid;
        cached_string_value = std::move(other.cached_string_value);
        
    }
    return *this;
}


value::value(std::monostate) : data(std::monostate{}) {}
value::value(const char* v) : data(std::string(v)) {}
value::value(std::string v) : data(std::move(v)) {}
value::value(int v) : data(static_cast<int64_t>(v)) {}
value::value(unsigned int v) : data(static_cast<int64_t>(v)) {}
value::value(int64_t v) : data(v) {}
value::value(double v) : data(v) {}
value::value(float v) : data(static_cast<double>(v)) {}
value::value(bool v) : data(v) {}
value::value(glm::vec2 v) : data(v) {}
value::value(glm::vec3 v) : data(v) {}
value::value(glm::vec4 v) : data(v) {}




bool value::isNull() const {
    return std::holds_alternative<std::monostate>(data);
}

bool value::isString() const {
    return std::holds_alternative<std::string>(data);
}

bool value::isNumber() const {
    return isInteger() || isFloat();
}

bool value::isInteger() const {
    return std::holds_alternative<int64_t>(data);
}

bool value::isFloat() const {
    return std::holds_alternative<double>(data);
}

bool value::isBool() const {
    return std::holds_alternative<bool>(data);
}

bool value::isVec2() const {
    return std::holds_alternative<glm::vec2>(data);
}

bool value::isVec3() const {
    return std::holds_alternative<glm::vec3>(data);
}

bool value::isVec4() const {
    return std::holds_alternative<glm::vec4>(data);
}

value::Type value::getType() const {
    if (isNull()) return Type::Null;
    if (isString()) return Type::String;
    if (isInteger()) return Type::Integer;
    if (isFloat()) return Type::Float;
    if (isBool()) return Type::Boolean;
    if (isVec2()) return Type::Vec2;
    if (isVec3()) return Type::Vec3;
    if (isVec4()) return Type::Vec4;
    return Type::Null; 
}





int64_t value::asInt(int64_t def) const {
    if (isInteger()) {
        return std::get<int64_t>(data);
    }
    if (isFloat()) {
        return static_cast<int64_t>(std::get<double>(data));
    }
    if (isString()) {
        if (auto parsed = parseInt(std::get<std::string>(data))) {
            return *parsed;
        }
    }
    return def;
}

double value::asFloat(double def) const {
    if (isFloat()) {
        return std::get<double>(data);
    }
    if (isInteger()) {
        return static_cast<double>(std::get<int64_t>(data));
    }
    if (isString()) {
        if (auto parsed = parseFloat(std::get<std::string>(data))) {
            return *parsed;
        }
    }
    return def;
}






value value::fromString(const std::string& str) {
    if (auto result = tryFromString(str)) {
        
        return *result;
    }
    return value(str); 
}

std::optional<value> value::tryFromString(const std::string& str) {
    if (str.empty()) return std::nullopt;

    
    if (auto val = parseInt(str)) {
        return value(*val);
    }

    if (auto val = parseFloat(str)) {
        return value(*val);
    }

    if (auto val = parseBool(str)) {
        return value(*val);
    }

    
    if (auto color = parseColor(str)) {
        return value(*color);
    }

    
    
    return std::nullopt;
}





std::optional<int64_t> value::parseInt(const std::string& str) {
    return parsers::parseInt(str);
}

std::optional<double> value::parseFloat(const std::string& str) {
    return parsers::parseFloat(str);
}

std::optional<bool> value::parseBool(const std::string& str) {
    return parsers::parseBool(str);
}

std::optional<glm::vec4> value::parseColor(const std::string& str) {
    return parsers::parseColor(str);
}


namespace parsers {
    std::optional<int64_t> parseInt(const std::string& str) {
        if (str.empty()) return std::nullopt;
        try {
            size_t pos;
            int64_t val = std::stoll(str, &pos);
            return pos == str.length() ? std::make_optional(val) : std::nullopt;
        } catch (...) {
            return std::nullopt;
        }
    }

    std::optional<double> parseFloat(const std::string& str) {
        if (str.empty()) return std::nullopt;
        try {
            size_t pos;
            double val = std::stod(str, &pos);
            return pos == str.length() ? std::make_optional(val) : std::nullopt;
        } catch (...) {
            return std::nullopt;
        }
    }

    std::optional<bool> parseBool(const std::string& str) {
        if (str == "true" || str == "1") return true;
        if (str == "false" || str == "0") return false;
        return std::nullopt;
    }

    std::optional<glm::vec4> parseColor(const std::string& str) {
        if (str.empty() || str[0] != '#') return std::nullopt;

        std::string hex = str.substr(1);

        
        hex.erase(std::remove(hex.begin(), hex.end(), ' '), hex.end());

        if (hex.length() == 3) {
            
            std::string expanded;
            for (char c : hex) {
                expanded += c;
                expanded += c;
            }
            hex = expanded;
        }

        if (hex.length() == 4) {
            
            std::string expanded;
            for (size_t i = 0; i < 4; ++i) {
                expanded += hex[i];
                expanded += hex[i];
            }
            hex = expanded;
        }

        if (hex.length() == 6) {
            
            try {
                unsigned int r = std::stoi(hex.substr(0, 2), nullptr, 16);
                unsigned int g = std::stoi(hex.substr(2, 2), nullptr, 16);
                unsigned int b = std::stoi(hex.substr(4, 2), nullptr, 16);
                return glm::vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
            } catch (...) {
                return std::nullopt;
            }
        }

        if (hex.length() == 8) {
            
            try {
                unsigned int r = std::stoi(hex.substr(0, 2), nullptr, 16);
                unsigned int g = std::stoi(hex.substr(2, 2), nullptr, 16);
                unsigned int b = std::stoi(hex.substr(4, 2), nullptr, 16);
                unsigned int a = std::stoi(hex.substr(6, 2), nullptr, 16);
                return glm::vec4(r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f);
            } catch (...) {
                return std::nullopt;
            }
        }

        return std::nullopt;
    }
} 





bool value::operator==(const value& other) const {
    
    if (this->getType() != other.getType()) return false;

    
    switch (this->getType()) {
        case Type::Null: return other.isNull();
        case Type::String: return std::get<std::string>(this->data) == std::get<std::string>(other.data);
        case Type::Integer: return std::get<int64_t>(this->data) == std::get<int64_t>(other.data);
        case Type::Float: return std::abs(std::get<double>(this->data) - std::get<double>(other.data)) < 1e-10; 
        case Type::Boolean: return std::get<bool>(this->data) == std::get<bool>(other.data);
        case Type::Vec2: return std::get<glm::vec2>(this->data) == std::get<glm::vec2>(other.data);
        case Type::Vec3: return std::get<glm::vec3>(this->data) == std::get<glm::vec3>(other.data);
        case Type::Vec4: return std::get<glm::vec4>(this->data) == std::get<glm::vec4>(other.data);
        default: return false; 
    }
}

bool value::operator!=(const value& other) const {
    return !(*this == other);
}

std::string value::typeName() const {
     switch (getType()) {
        case Type::Null: return "null";
        case Type::String: return "string";
        case Type::Integer: return "integer";
        case Type::Float: return "float";
        case Type::Boolean: return "boolean";
        case Type::Vec2: return "vec2";
        case Type::Vec3: return "vec3";
        case Type::Vec4: return "vec4";
        default: return "unknown";
    }
}


std::size_t value::Hash::operator()(const value& v) const {
    
    switch (v.getType()) {
        case Type::String:
            return std::hash<std::string>{}(v.asString());
        case Type::Integer:
            return std::hash<int64_t>{}(v.asInt());
        case Type::Float:
            return std::hash<double>{}(v.asFloat(0.0));
        case Type::Boolean:
            return std::hash<bool>{}(v.asBool());
        case Type::Vec2: {
             auto vec = v.asVec2();
             return std::hash<float>{}(vec.x) ^ (std::hash<float>{}(vec.y) << 1);
        }
        case Type::Vec3: {
             auto vec = v.asVec3();
             return std::hash<float>{}(vec.x) ^ (std::hash<float>{}(vec.y) << 1) ^ (std::hash<float>{}(vec.z) << 2);
        }
        case Type::Vec4: {
             auto vec = v.asVec4();
             return std::hash<float>{}(vec.x) ^ (std::hash<float>{}(vec.y) << 1) ^ (std::hash<float>{}(vec.z) << 2) ^ (std::hash<float>{}(vec.w) << 3);
        }
        default:
            return 0;
    }
}


bool value::asBool(bool def) const {
    if (isBool()) {
        return std::get<bool>(data);
    }

    return def;
}

std::string value::asString(const std::string& def) const {
    if (isString()) {
        return std::get<std::string>(data);
    }

    if (isNull()) {
        return def; 
    }

    return def;
}

glm::vec2 value::asVec2(const glm::vec2& def) const {
    if (isVec2()) {
        return std::get<glm::vec2>(data);
    }

    return def;
}

glm::vec3 value::asVec3(const glm::vec3& def) const {
    if (isVec3()) {
        return std::get<glm::vec3>(data);
    }

    if (isVec2()) {
        auto v2 = std::get<glm::vec2>(data);
        return glm::vec3(v2, 0.0f);
    }

    return def;
}

glm::vec4 value::asVec4(const glm::vec4& def) const {
    if (isVec4()) {
        return std::get<glm::vec4>(data);
    }
    
    if (isVec3()) {
        auto v3 = std::get<glm::vec3>(data);
        return glm::vec4(v3, 1.0f);
    }
    
    if (isVec2()) {
        auto v2 = std::get<glm::vec2>(data);
        return glm::vec4(v2, 0.0f, 1.0f);
    }

    return def;
}


}