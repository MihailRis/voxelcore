#include "PostEffect.hpp"

#include "Shader.hpp"
#include "data/dv_util.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("post-effect");

PostEffect::Param::Param() : type(Type::FLOAT) {}

PostEffect::Param::Param(Type type, Value defValue, bool array)
    : type(type), defValue(defValue), value(defValue), array(array) {
}

PostEffect::PostEffect(
    bool advanced,
    std::shared_ptr<Shader> shader,
    std::unordered_map<std::string, Param> params
)
    : advanced(advanced), shader(std::move(shader)), params(std::move(params)) {
}

Shader& PostEffect::use() {
    shader->use();
    shader->uniform1f("u_intensity", intensity);
    for (auto& [name, param] : params) {
        if (!param.dirty) {
            continue;
        }
        if (param.array) {
            const auto& found = arrayValues.find(name);
            if (found == arrayValues.end()) {
                continue;
            }
            size_t size = found->second.size();
            auto ibuffer = reinterpret_cast<const int*>(found->second.data());
            auto fbuffer = reinterpret_cast<const float*>(found->second.data());
            switch (param.type) {
                case Param::Type::INT:
                    shader->uniform1v(name, size / sizeof(int), ibuffer);
                    break;
                case Param::Type::FLOAT:
                    shader->uniform1v(name, size / sizeof(float), fbuffer);
                    break;
                case Param::Type::VEC2:
                    shader->uniform2v(name, size / sizeof(glm::vec2), fbuffer);
                    break;
                case Param::Type::VEC3:
                    shader->uniform3v(name, size / sizeof(glm::vec3), fbuffer);
                    break;
                case Param::Type::VEC4:
                    shader->uniform4v(name, size / sizeof(glm::vec4), fbuffer);
                    break;
                default:
                    assert(false);
            }
        } else {
            switch (param.type) {
                case Param::Type::INT:
                    shader->uniform1i(name, std::get<int>(param.value));
                    break;
                case Param::Type::FLOAT:
                    shader->uniform1f(name, std::get<float>(param.value));
                    break;
                case Param::Type::VEC2:
                    shader->uniform2f(name, std::get<glm::vec2>(param.value));
                    break;
                case Param::Type::VEC3:
                    shader->uniform3f(name, std::get<glm::vec3>(param.value));
                    break;
                case Param::Type::VEC4:
                    shader->uniform4f(name, std::get<glm::vec4>(param.value));
                    break;
                default:
                    assert(false);
            }
        }
        param.dirty = false;
    }
    return *shader;
}

float PostEffect::getIntensity() const {
    return intensity;
}

void PostEffect::setIntensity(float value) {
    intensity = value;
}

template<int n>
static void set_value(PostEffect::Param::Value& dst, const dv::value& value) {
    glm::vec<n, float> vec;
    dv::get_vec(value, vec);
    dst = vec;
}

void PostEffect::setParam(const std::string& name, const dv::value& value) {
    const auto& found = params.find(name);
    if (found == params.end()) {
        return;
    }
    auto& param = found->second;
    switch (param.type) {
        case Param::Type::INT:
            param.value = static_cast<int>(value.asInteger());
            break;
        case Param::Type::FLOAT:
            param.value = static_cast<float>(value.asNumber());
            break;
        case Param::Type::VEC2:
            set_value<2>(param.value, value);
            break;
        case Param::Type::VEC3:
            set_value<3>(param.value, value);
            break;
        case Param::Type::VEC4:
            set_value<4>(param.value, value);
            break;
    }
    param.dirty = true;
}

void PostEffect::setArray(const std::string& name, std::vector<ubyte>&& values) {
    const auto& found = params.find(name);
    if (found == params.end()) {
        return;
    }
    auto& param = found->second;
    if (!param.array) {
        logger.warning() << "set_array is used on non-array effect parameter";
        if (!values.empty()) {
            setParam(name, values[0]);
        }
        return;
    }
    param.dirty = true;
    arrayValues[name] = std::move(values);
}
