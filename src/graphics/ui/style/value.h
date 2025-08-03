
#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

namespace style {

    class value {
    public:
        enum class Type {
            Null,
            String,
            Integer,
            Float,
            Boolean,
            Vec2,
            Vec3,
            Vec4
        };
    private:
        std::variant<
            std::monostate,
            std::string,
            int64_t,
            double,
            bool,
            glm::vec2,
            glm::vec3,
            glm::vec4>
            data;

        mutable std::unique_ptr<glm::vec4> color_cache;
        mutable bool color_cache_valid = false;
        mutable std::string cached_string_value;

        static std::optional<int64_t> parseInt(const std::string& str);
        static std::optional<double> parseFloat(const std::string& str);
        static std::optional<bool> parseBool(const std::string& str);
        static std::optional<glm::vec4> parseColor(const std::string& str);
    public:
        value();
        value(const value& other);
        value& operator=(const value& other);
        value(value&& other) noexcept;
        value& operator=(value&& other) noexcept;

        value(std::monostate);
        value(const char* v);
        value(std::string v);
        value(int v);
        value(unsigned int v);
        value(int64_t v);
        value(double v);
        value(float v);
        value(bool v);
        value(glm::vec2 v);
        value(glm::vec3 v);
        value(glm::vec4 v);

        static value fromString(const std::string& str);
        static std::optional<value> tryFromString(const std::string& str);

        bool isNull() const;
        bool isString() const;
        bool isNumber() const;
        bool isInteger() const;
        bool isFloat() const;
        bool isBool() const;
        bool isVec2() const;
        bool isVec3() const;
        bool isVec4() const;

        Type getType() const;

        int64_t asInt(int64_t def = 0) const;
        double asFloat(double def = 0.0) const;
        float asFloat(float def = 0.0f) const;
        bool asBool(bool def = false) const;
        std::string asString(const std::string& def = "") const;
        glm::vec2 asVec2(const glm::vec2& def = {}) const;
        glm::vec3 asVec3(const glm::vec3& def = {}) const;
        glm::vec4 asVec4(const glm::vec4& def = {}) const;
        glm::vec4 asColor(const glm::vec4& def = {0, 0, 0, 1}) const;

        std::string typeName() const;
        std::string toString() const;

        bool operator==(const value& other) const;
        bool operator!=(const value& other) const;

        struct Hash {
            std::size_t operator()(const value& v) const;
        };
    };

    namespace parsers {
        std::optional<int64_t> parseInt(const std::string& str);
        std::optional<double> parseFloat(const std::string& str);
        std::optional<bool> parseBool(const std::string& str);
        std::optional<glm::vec4> parseColor(const std::string& str);
    }

}