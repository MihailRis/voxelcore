#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <variant>
#include <optional>
#include <sstream>
#include <cctype>

namespace style {

/**
 * Универсальный тип для хранения динамических значений.
 * Поддерживает: строки, числа, bool, vec2/3/4, цвета.
 */
class value {
private:
    std::variant<
        std::monostate, // null
        std::string,
        int64_t,
        double,
        bool,
        glm::vec2,
        glm::vec3,
        glm::vec4
    > data;

    // Вспомогательная функция для парсинга строки в число
    template<typename T>
    static std::optional<T> tryParse(const std::string& str);

public:
    // Конструкторы
    value() = default;
    value(std::monostate) {}
    value(const char* v) : data(std::string(v)) {}
    value(std::string v) : data(std::move(v)) {}
    value(int v) : data(static_cast<int64_t>(v)) {}
    value(unsigned int v) : data(static_cast<int64_t>(v)) {}
    value(int64_t v) : data(v) {}
    value(double v) : data(v) {}
    value(float v) : data(static_cast<double>(v)) {}
    value(bool v) : data(v) {}
    value(glm::vec2 v) : data(v) {}
    value(glm::vec3 v) : data(v) {}
    value(glm::vec4 v) : data(v) {}

    // Явные преобразования
    bool isNull() const { return std::holds_alternative<std::monostate>(data); }
    bool isString() const { return std::holds_alternative<std::string>(data); }
    bool isNumber() const { return isInteger() || isFloat(); }
    bool isInteger() const { return std::holds_alternative<int64_t>(data); }
    bool isFloat() const { return std::holds_alternative<double>(data); }
    bool isBool() const { return std::holds_alternative<bool>(data); }
    bool isVec2() const { return std::holds_alternative<glm::vec2>(data); }
    bool isVec3() const { return std::holds_alternative<glm::vec3>(data); }
    bool isVec4() const { return std::holds_alternative<glm::vec4>(data); }

    // Получение значений с преобразованием
    int64_t asInt(int64_t def = 0) const;
    double asFloat(double def = 0.0) const;
    float asFloat(float def = 0.0f) const;
    bool asBool(bool def = false) const;
    std::string asString(const std::string& def = "") const;

    glm::vec2 asVec2(const glm::vec2& def = {}) const;
    glm::vec3 asVec3(const glm::vec3& def = {}) const;
    glm::vec4 asVec4(const glm::vec4& def = {}) const;

    // Специальный: цвет (поддерживает #RGB, #RGBA, #RRGGBB, rgba())
    glm::vec4 asColor(const glm::vec4& def = {0,0,0,0}) const;

    // Для отладки
    std::string type_name() const;
    std::string toString() const;
};

// Реализация шаблонов
template<typename T>
std::optional<T> value::tryParse(const std::string& str) {
    if (str.empty()) return std::nullopt;
    try {
        size_t pos;
        if constexpr (std::is_same_v<T, int64_t>) {
            int64_t val = std::stoll(str, &pos);
            return pos == str.length() ? std::make_optional(val) : std::nullopt;
        } else if constexpr (std::is_same_v<T, double>) {
            double val = std::stod(str, &pos);
            return pos == str.length() ? std::make_optional(val) : std::nullopt;
        } else if constexpr (std::is_same_v<T, bool>) {
            if (str == "true" || str == "1") return true;
            if (str == "false" || str == "0") return false;
            return std::nullopt;
        }
    } catch (...) {
        return std::nullopt;
    }
    return std::nullopt;
}

}