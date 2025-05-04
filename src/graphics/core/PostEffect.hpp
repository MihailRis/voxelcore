#pragma once

#include <memory>
#include <string>
#include <variant>
#include <unordered_map>
#include <glm/glm.hpp>

#include "data/dv_fwd.hpp"
#include "util/EnumMetadata.hpp"

class Shader;

class PostEffect {
public:
    struct Param {
        enum class Type { FLOAT, VEC2, VEC3, VEC4 };

        VC_ENUM_METADATA(Type)
            {"float", Type::FLOAT},
            {"vec2", Type::VEC2},
            {"vec3", Type::VEC3},
            {"vec4", Type::VEC4},
        VC_ENUM_END

        using Value = std::variant<float, glm::vec2, glm::vec3, glm::vec4>;

        Type type;
        Value defValue;
        Value value;
        bool dirty = true;

        Param();
        Param(Type type, Value defValue);
    };

    PostEffect(
        std::shared_ptr<Shader> shader,
        std::unordered_map<std::string, Param> params
    );

    explicit PostEffect(const PostEffect&) = default;

    Shader& use();

    float getIntensity() const;
    void setIntensity(float value);

    void setParam(const std::string& name, const dv::value& value);

    bool isActive() {
        return intensity > 1e-4f;
    }
private:
    std::shared_ptr<Shader> shader;
    std::unordered_map<std::string, Param> params;
    float intensity = 0.0f;
};
