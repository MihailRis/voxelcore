#pragma once

#include "typedefs.hpp"

#include <string>
#include <memory>
#include <unordered_map>
#include <glm/glm.hpp>

class GLSLExtension;

class Shader {
public:
    struct Source {
        std::string file;
        std::string code;
    };
private:
    static Shader* used;
    uint id;
    std::unordered_map<std::string, uint> uniformLocations;
    
    // source code used for re-compiling shaders after updating defines
    Source vertexSource;
    Source fragmentSource;
    
    uint getUniformLocation(const std::string& name);
public:
    static GLSLExtension* preprocessor;

    Shader(uint id, Source&& vertexSource, Source&& fragmentSource);
    ~Shader();

    void use();
    void uniformMatrix(const std::string&, const glm::mat4& matrix);
    void uniformMatrix(const std::string&, const glm::mat3& matrix);
    void uniform1i(const std::string& name, int x);
    void uniform1f(const std::string& name, float x);
    void uniform2f(const std::string& name, float x, float y);
    void uniform2f(const std::string& name, const glm::vec2& xy);
    void uniform2i(const std::string& name, const glm::ivec2& xy);
    void uniform3f(const std::string& name, float x, float y, float z);
    void uniform3f(const std::string& name, const glm::vec3& xyz);
    void uniform4f(const std::string& name, const glm::vec4& xyzw);

    void uniform1v(const std::string& name, int length, const int* v);
    void uniform1v(const std::string& name, int length, const float* v);
    void uniform2v(const std::string& name, int length, const float* v);
    void uniform3v(const std::string& name, int length, const float* v);
    void uniform4v(const std::string& name, int length, const float* v);

    /// @brief Re-preprocess source code and re-compile shader program
    void recompile();

    /// @brief Create shader program using vertex and fragment shaders source.
    /// @return linked shader program containing vertex and fragment shaders
    static std::unique_ptr<Shader> create(
        Source&& vertexSource, Source&& fragmentSource
    );

    static Shader& getUsed();
};
