#define VC_ENABLE_REFLECTION
#include "GLSLExtension.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

#include "debug/Logger.hpp"
#include "io/engine_paths.hpp"
#include "typedefs.hpp"
#include "util/stringutil.hpp"
#include "coders/json.hpp"
#include "data/dv_util.hpp"
#include "coders/BasicParser.hpp"

static debug::Logger logger("glsl-extension");

using Type = PostEffect::Param::Type;
using Value = PostEffect::Param::Value;

void GLSLExtension::setPaths(const ResPaths* paths) {
    this->paths = paths;
}

void GLSLExtension::loadHeader(const std::string& name) {
    if (paths == nullptr) {
        return;
    }
    io::path file = paths->find("shaders/lib/" + name + ".glsl");
    std::string source = io::read_string(file);
    addHeader(name, {});
    addHeader(name, process(file, source, true));
}

void GLSLExtension::addHeader(const std::string& name, ProcessingResult header) {
    headers[name] = std::move(header);
}

void GLSLExtension::define(const std::string& name, std::string value) {
    defines[name] = std::move(value);
}

const GLSLExtension::ProcessingResult& GLSLExtension::getHeader(
    const std::string& name
) const {
    auto found = headers.find(name);
    if (found == headers.end()) {
        throw std::runtime_error("no header '" + name + "' loaded");
    }
    return found->second;
}

const std::string& GLSLExtension::getDefine(const std::string& name) const {
    auto found = defines.find(name);
    if (found == defines.end()) {
        throw std::runtime_error("name '" + name + "' is not defined");
    }
    return found->second;
}

const std::unordered_map<std::string, std::string>& GLSLExtension::getDefines() const {
    return defines;
}

bool GLSLExtension::hasDefine(const std::string& name) const {
    return defines.find(name) != defines.end();
}

bool GLSLExtension::hasHeader(const std::string& name) const {
    return headers.find(name) != headers.end();
}

void GLSLExtension::undefine(const std::string& name) {
    if (hasDefine(name)) {
        defines.erase(name);
    }
}

void GLSLExtension::setDefined(const std::string& name, bool defined) {
    if (defined) {
        define(name, "TRUE");
    } else {
        undefine(name);
    }
}

inline std::runtime_error parsing_error(
    const io::path& file, uint linenum, const std::string& message
) {
    return std::runtime_error(
        "file " + file.string() + ": " + message + " at line " +
        std::to_string(linenum)
    );
}

inline void parsing_warning(
    std::string_view file, uint linenum, const std::string& message
) {
    logger.warning() << "file " + std::string(file) + ": warning: " + message +
                     " at line " + std::to_string(linenum);
}

inline void source_line(std::stringstream& ss, uint linenum) {
    ss << "#line " << linenum << "\n";
}

static Value default_value_for(Type type) {
    switch (type) {
        case Type::INT:
            return 0;
        case Type::FLOAT:
            return 0.0f;
        case Type::VEC2:
            return glm::vec2 {0.0f, 0.0f};
        case Type::VEC3:
            return glm::vec3 {0.0f, 0.0f, 0.0f};
        case Type::VEC4:
            return glm::vec4 {0.0f, 0.0f, 0.0f, 0.0f};
        default:
            throw std::runtime_error("unsupported type");
    }
}

class GLSLParser : public BasicParser<char> {
public:
    GLSLParser(GLSLExtension& glsl, std::string_view file, std::string_view source, bool header)
        : BasicParser(file, source), glsl(glsl) {
        if (!header) {
            ss << "#version " << GLSLExtension::VERSION << '\n';
        }
        for (auto& entry : glsl.getDefines()) {
            ss << "#define " << entry.first << " " << entry.second << '\n';
        }
        uint linenum = 1;
        source_line(ss, linenum);

        clikeComment = true;
    }

    bool processIncludeDirective() {
        skipWhitespace(false);
        if (peekNoJump() != '<') {
            throw error("'<' expected");
        }
        skip(1);
        skipWhitespace(false);
        auto headerName = parseName();
        skipWhitespace(false);
        if (peekNoJump() != '>') {
            throw error("'>' expected");
        }
        skip(1);
        skipWhitespace(false);
        skipLine();

        if (!glsl.hasHeader(headerName)) {
            glsl.loadHeader(headerName);
        }
        const auto& header = glsl.getHeader(headerName);
        for (const auto& [name, param] : header.params) {
            params[name] = param;
        }
        ss << header.code << '\n';
        source_line(ss, line);
        return false;
    }

    bool processVersionDirective() {
        source_line(ss, line);
        skipLine();
        return false;
    }

    template<int n>
    Value parseVectorValue() {
        if (peekNoJump() != '[') {
            throw error("'[' expected");
        }
        // may be more efficient but ok
        auto value = json::parse(
            filename,
            std::string_view(source.data() + pos, source.size() - pos)
        );
        glm::vec<n, float> vec {};
        try {
            dv::get_vec<n>(value, vec);
            return vec;
        } catch (const std::exception& err) {
            throw error(err.what());
        }
    }

    Value parseDefaultValue(Type type, const std::string& name) {
        switch (type) {
            case Type::INT:
                return static_cast<int>(parseNumber(1).asInteger());
            case Type::FLOAT:
                return static_cast<float>(parseNumber(1).asNumber());
            case Type::VEC2:
                return parseVectorValue<2>();
            case Type::VEC3:
                return parseVectorValue<3>();
            case Type::VEC4:
                return parseVectorValue<4>();
            default:
                throw error("unsupported default value for type " + name);
        }
    }

    bool processParamDirective() {
        using Param = PostEffect::Param;

        skipWhitespace(false);
        // Parse type name
        auto typeName = parseName();
        Param::Type type {};
        if (!Param::TypeMeta.getItem(typeName, type)) {
            throw error("unsupported param type " + util::quote(typeName));
        }
        skipWhitespace(false);
        // Parse parameter name
        auto paramName = parseName();
        if (params.find(paramName) != params.end()) {
            throw error("duplicating param " + util::quote(paramName));
        }
        
        skipWhitespace(false);
        int start = pos;

        ss << "uniform " << typeName << " " << paramName;

        bool array = false;
        if (peekNoJump() == '[') {
            skip(1);
            array = true;
            readUntil(']');
            skip(1);
            ss << source.substr(start, pos - start + 1);
        }

        ss << ";\n";
        
        auto defValue = default_value_for(type);
        // Parse default value
        if (peekNoJump() == '=') {
            skip(1);
            skipWhitespace(false);
            defValue = parseDefaultValue(type, typeName);
        }

        skipLine();

        params[paramName] = PostEffect::Param(type, std::move(defValue), array);
        return false;
    }

    bool processPreprocessorDirective() {
        skip(1);

        auto name = parseName();

        if (name == "version") {
            return processVersionDirective();
        } else if (name == "include") {
            return processIncludeDirective();
        } else if (name == "param") {
            return processParamDirective();
        }
        return true;
    }

    GLSLExtension::ProcessingResult process() {
        while (hasNext()) {
            skipWhitespace(false);
            if (!hasNext()) {
                break;
            }
            if (source[pos] != '#' || processPreprocessorDirective()) {
                pos = linestart;
                ss << readUntilEOL() << '\n';
                skip(1);
            }
        }
        return {ss.str(), std::move(params)};
    }
private:
    GLSLExtension& glsl;
    std::unordered_map<std::string, PostEffect::Param> params;
    std::stringstream ss;
};

GLSLExtension::ProcessingResult GLSLExtension::process(
    const io::path& file, const std::string& source, bool header
) {
    std::string filename = file.string();
    GLSLParser parser(*this, filename, source, header);
    return parser.process();
}
