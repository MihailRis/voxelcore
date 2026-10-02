#include "MaterialShaders.hpp"

#include <cctype>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "coders/GLSLExtension.hpp"
#include "content/Content.hpp"
#include "data/dv.hpp"
#include "debug/Logger.hpp"
#include "engine/EnginePaths.hpp"
#include "graphics/core/Shader.hpp"
#include "io/io.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"

static debug::Logger logger("material-shaders");

namespace {
    struct MaterialShader {
        std::string name;
        uint8_t index;
        GLSLExtension::ParamsMap params;
        std::vector<std::string> materials;

        std::string prefix() const {
            return "material_" + std::to_string(index) + "_";
        }
    };

    struct Stage {
        const char* header;
        const char* extension;
        const char* function;
        /// @brief hook argument type and name
        const char* type;
        const char* arg;
    };

    const Stage STAGES[] {
        {material_shaders::VERTEX_HEADER,
         ".glslv",
         "material_vertex",
         "vec3",
         "position"},
        {material_shaders::FRAGMENT_HEADER,
         ".glslf",
         "material_fragment",
         "vec4",
         "color"},
    };

    /// @brief shader index -> shader
    std::map<uint8_t, MaterialShader> shaders;
    /// @brief material name -> shader index
    std::unordered_map<std::string, uint8_t> materials;
    /// @brief source index of the generated dispatch code in GLSL errors
    const int GENERATED_SOURCE = 128;
    /// @brief incremented on any params change
    uint64_t paramsVersion = 1;

    struct UploadState {
        uint program;
        uint64_t version;
    };
    /// @brief shader -> uploaded params state
    std::unordered_map<const Shader*, UploadState> uploaded;
}

static bool is_identifier_char(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

/// @brief Remove comments from GLSL code
static std::string strip_comments(const std::string& code) {
    std::string out;
    out.reserve(code.size());
    for (size_t i = 0; i < code.size(); i++) {
        if (code.compare(i, 2, "//") == 0) {
            i = code.find('\n', i);
            if (i == std::string::npos) {
                break;
            }
        } else if (code.compare(i, 2, "/*") == 0) {
            i = code.find("*/", i + 2);
            if (i == std::string::npos) {
                break;
            }
            i++;
            continue;
        }
        out += code[i];
    }
    return out;
}

/// @brief Check if the function is defined in the code
static bool has_function(const std::string& source, const std::string& name) {
    std::string code = strip_comments(source);
    size_t pos = 0;
    while ((pos = code.find(name, pos)) != std::string::npos) {
        size_t end = pos + name.size();
        bool word = (pos == 0 || !is_identifier_char(code[pos - 1])) &&
                    (end == code.size() || !is_identifier_char(code[end]));
        pos = end;
        if (!word) {
            continue;
        }
        while (end < code.size() &&
               std::isspace(static_cast<unsigned char>(code[end]))) {
            end++;
        }
        if (end < code.size() && code[end] == '(') {
            return true;
        }
    }
    return false;
}

/// @brief Include the material shader stage renaming the hook function and
/// the params to unique names (GLSL has no namespaces)
/// @return true if the stage file exists
static bool include_shader(
    GLSLExtension& preprocessor,
    const ResPaths& paths,
    MaterialShader& shader,
    const Stage& stage,
    std::stringstream& ss
) {
    io::path file =
        paths.find("shaders/materials/" + shader.name + stage.extension);
    if (!io::exists(file)) {
        return false;
    }
    auto result = preprocessor.process(file, io::read_string(file), true, {});
    std::string prefix = shader.prefix();

    ss << "// " << shader.name << " (" << file.string() << ")\n";
    // compilation errors refer to the shader by its index: <index>:<line>
    ss << "#line 1 " << static_cast<int>(shader.index) << "\n";
    ss << "#define " << stage.function << " " << prefix << stage.function
       << "\n";
    for (const auto& [name, param] : result.params) {
        if (param.array) {
            throw std::runtime_error(
                file.string() + ": array params are not supported"
            );
        }
        if (name[0] == '_') {
            throw std::runtime_error(
                file.string() +
                ": param name must not start with '_': " + util::quote(name)
            );
        }
        const auto& found = shader.params.find(name);
        if (found != shader.params.end() && found->second.type != param.type) {
            throw std::runtime_error(
                file.string() + ": param " + util::quote(name) +
                " type differs from the other stage"
            );
        }
        ss << "#define " << name << " " << prefix << name << "\n";
        shader.params[name] = param;
    }
    ss << result.code << "\n";
    ss << "#undef " << stage.function << "\n";
    for (const auto& entry : result.params) {
        ss << "#undef " << entry.first << "\n";
    }
    if (!has_function(result.code, stage.function)) {
        throw std::runtime_error(
            file.string() + ": function " + stage.function + " is not defined"
        );
    }
    return true;
}

static void build_stage_header(
    GLSLExtension& preprocessor, const ResPaths& paths, const Stage& stage
) {
    std::stringstream ss;
    std::vector<const MaterialShader*> hooked;

    for (auto& [index, shader] : shaders) {
        if (include_shader(preprocessor, paths, shader, stage, ss)) {
            hooked.push_back(&shader);
        }
    }
    // generated code source index, then back to the host shader source
    ss << "#line 1 " << GENERATED_SOURCE << "\n";
    ss << stage.type << " apply_" << stage.function << "(int material, "
       << stage.type << " " << stage.arg << ") {\n";
    if (!hooked.empty()) {
        ss << "    switch (material) {\n";
        for (const auto* shader : hooked) {
            ss << "        case " << static_cast<int>(shader->index)
               << ": return " << shader->prefix() << stage.function << "("
               << stage.arg << ");\n";
        }
        ss << "    }\n";
    }
    ss << "    return " << stage.arg << ";\n";
    ss << "}\n";
    ss << "#line 1 0\n";

    preprocessor.addHeader(stage.header, {ss.str(), {}});
}

void material_shaders::build_headers(
    GLSLExtension& preprocessor, const ResPaths& paths, const Content* content
) {
    shaders.clear();
    materials.clear();
    uploaded.clear();
    paramsVersion++;

    if (content) {
        for (const auto& [name, material] : content->getBlockMaterials()) {
            uint8_t index = material->rt.shaderId;
            if (index == 0) {
                continue;
            }
            auto& shader = shaders[index];
            shader.name = material->shader;
            shader.index = index;
            shader.materials.push_back(name);
            materials[name] = index;
        }
    }
    for (const auto& stage : STAGES) {
        build_stage_header(preprocessor, paths, stage);
    }
    for (const auto& [index, shader] : shaders) {
        bool found = false;
        for (const auto& stage : STAGES) {
            found |= io::exists(
                paths.find("shaders/materials/" + shader.name + stage.extension)
            );
        }
        if (!found) {
            logger.warning() << "material shader " << util::quote(shader.name)
                             << " files not found";
        }
        logger.info() << "material shader " << static_cast<int>(index) << ": "
                      << shader.name << " ("
                      << util::join(shader.materials, ',') << ")";
    }
}

void material_shaders::setup_shader(Shader& shader) {
    auto& state = uploaded[&shader];
    if (state.program == shader.getId() && state.version == paramsVersion) {
        return;
    }
    for (const auto& [index, materialShader] : shaders) {
        for (const auto& [name, param] : materialShader.params) {
            param.apply(shader, materialShader.prefix() + name);
        }
    }
    state = {shader.getId(), paramsVersion};
}

static MaterialShader& require_shader(const std::string& material) {
    const auto& found = materials.find(material);
    if (found == materials.end()) {
        throw std::runtime_error(
            "material " + util::quote(material) + " has no shader"
        );
    }
    return shaders.at(found->second);
}

void material_shaders::set_params(
    const std::string& material, const dv::value& params
) {
    auto& shader = require_shader(material);
    // convert all values before updating
    GLSLExtension::ParamsMap updated;
    for (const auto& [name, value] : params.asObject()) {
        const auto& found = shader.params.find(name);
        if (found == shader.params.end()) {
            throw std::runtime_error(
                "material " + util::quote(material) + " has no param " +
                util::quote(name)
            );
        }
        auto param = found->second;
        try {
            param.set(value);
        } catch (const std::exception& err) {
            throw std::runtime_error(
                "material " + util::quote(material) + " param " +
                util::quote(name) + ": " + err.what()
            );
        }
        updated[name] = std::move(param);
    }
    for (auto& [name, param] : updated) {
        shader.params[name] = param;
    }
    paramsVersion++;
}

dv::value material_shaders::get_params(const std::string& material) {
    const auto& found = materials.find(material);
    if (found == materials.end()) {
        return nullptr;
    }
    auto table = dv::object();
    for (const auto& [name, param] : shaders.at(found->second).params) {
        table[name] = param.get();
    }
    return table;
}
