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

void PostEffect::Param::apply(Shader& shader, const std::string& name) const {
    switch (type) {
        case Type::INT:
            shader.uniform1i(name, std::get<int>(value));
            break;
        case Type::FLOAT:
            shader.uniform1f(name, std::get<float>(value));
            break;
        case Type::VEC2:
            shader.uniform2f(name, std::get<glm::vec2>(value));
            break;
        case Type::VEC3:
            shader.uniform3f(name, std::get<glm::vec3>(value));
            break;
        case Type::VEC4:
            shader.uniform4f(name, std::get<glm::vec4>(value));
            break;
        default:
            assert(false);
    }
}

static void apply_uniform_array(
    const PostEffect::Param& param,
    Shader& shader,
    const std::string& name,
    const std::vector<ubyte>& values
) {
    size_t size = values.size();
    auto ibuffer = reinterpret_cast<const int*>(values.data());
    auto fbuffer = reinterpret_cast<const float*>(values.data());

    using Type = PostEffect::Param::Type;
    switch (param.type) {
        case Type::INT:
            shader.uniform1v(name, size / sizeof(int), ibuffer);
            break;
        case Type::FLOAT:
            shader.uniform1v(name, size / sizeof(float), fbuffer);
            break;
        case Type::VEC2:
            shader.uniform2v(name, size / sizeof(glm::vec2), fbuffer);
            break;
        case Type::VEC3:
            shader.uniform3v(name, size / sizeof(glm::vec3), fbuffer);
            break;
        case Type::VEC4:
            shader.uniform4v(name, size / sizeof(glm::vec4), fbuffer);
            break;
        default:
            assert(false);
    }
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
            apply_uniform_array(param, *shader, name, found->second);
        } else {
            param.apply(*shader, name);
        }
        param.dirty = false;
    }
    return *shader;
}

Shader& PostEffect::getShader() {
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

void PostEffect::Param::set(const dv::value& src) {
    switch (type) {
        case Type::INT:
            value = static_cast<int>(src.asInteger());
            break;
        case Type::FLOAT:
            value = static_cast<float>(src.asNumber());
            break;
        case Type::VEC2:
            set_value<2>(value, src);
            break;
        case Type::VEC3:
            set_value<3>(value, src);
            break;
        case Type::VEC4:
            set_value<4>(value, src);
            break;
    }
    dirty = true;
}

dv::value PostEffect::Param::get() const {
    switch (type) {
        case Type::INT:
            return std::get<int>(value);
        case Type::FLOAT:
            return std::get<float>(value);
        case Type::VEC2:
            return dv::to_value(std::get<glm::vec2>(value));
        case Type::VEC3:
            return dv::to_value(std::get<glm::vec3>(value));
        case Type::VEC4:
            return dv::to_value(std::get<glm::vec4>(value));
    }
    return nullptr;
}

void PostEffect::setParam(const std::string& name, const dv::value& value) {
    const auto& found = params.find(name);
    if (found == params.end()) {
        return;
    }
    found->second.set(value);
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
