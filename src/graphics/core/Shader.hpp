#pragma once

#include "typedefs.hpp"

#include <string>
#include <memory>
#include <unordered_map>
#include <glm/glm.hpp>

class GLSLExtension;

class Shader {
    static Shader* used;
    uint id;
    std::unordered_map<std::string, uint> uniformLocations;
    
    uint getUniformLocation(const std::string& name);
public:
    static GLSLExtension* preprocessor;

    Shader(uint id);
    ~Shader();

    void use();
    void uniformMatrix(const std::string&, const glm::mat4& matrix);
    void uniform1i(const std::string& name, int x);
    void uniform1f(const std::string& name, float x);
    void uniform2f(const std::string& name, float x, float y);
    void uniform2f(const std::string& name, const glm::vec2& xy);
    void uniform2i(const std::string& name, const glm::ivec2& xy);
    void uniform3f(const std::string& name, float x, float y, float z);
    void uniform3f(const std::string& name, const glm::vec3& xyz);
    void uniform4f(const std::string& name, const glm::vec4& xyzw);

    /// @brief Create shader program using vertex and fragment shaders source.
    /// @param vertexFile vertex shader file name
    /// @param fragmentFile fragment shader file name
    /// @param vertexSource vertex shader source code
    /// @param fragmentSource fragment shader source code
    /// @return linked shader program containing vertex and fragment shaders
    static std::unique_ptr<Shader> create(
        const std::string& vertexFile, 
        const std::string& fragmentFile,
        const std::string& vertexSource, 
        const std::string& fragmentSource
    );

    static Shader& getUsed();
};
