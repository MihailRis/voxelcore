#pragma once

#include <vector>
#include <memory>
#include <string>
#include <variant>
#include <unordered_map>
#include <glm/glm.hpp>

#include "typedefs.hpp"
#include "data/dv_fwd.hpp"
#include "util/EnumMetadata.hpp"

class Shader;

class PostEffect {
public:
    struct Param {
        enum class Type { INT, FLOAT, VEC2, VEC3, VEC4 };

        VC_ENUM_METADATA(Type)
            {"int", Type::INT},
            {"float", Type::FLOAT},
            {"vec2", Type::VEC2},
            {"vec3", Type::VEC3},
            {"vec4", Type::VEC4},
        VC_ENUM_END

        using Value = std::variant<int, float, glm::vec2, glm::vec3, glm::vec4>;

        Type type;
        Value defValue;
        Value value;
        bool array = false;
        bool dirty = true;

        Param();
        Param(Type type, Value defValue, bool array);
    };

    PostEffect(
        bool advanced,
        std::shared_ptr<Shader> shader,
        std::unordered_map<std::string, Param> params
    );

    explicit PostEffect(const PostEffect&) = default;

    Shader& use();

    Shader& getShader();

    float getIntensity() const;
    void setIntensity(float value);

    void setParam(const std::string& name, const dv::value& value);

    void setArray(const std::string& name, std::vector<ubyte>&& values);

    bool isAdvanced() const {
        return advanced;
    }

    bool isActive() {
        return intensity > 1e-4f;
    }
private:
    bool advanced = false;
    std::shared_ptr<Shader> shader;
    std::unordered_map<std::string, Param> params;
    std::unordered_map<std::string, std::vector<ubyte>> arrayValues;
    float intensity = 0.0f;
};
