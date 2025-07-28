#include "Shader.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <filesystem>

#include <glm/gtc/type_ptr.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "coders/GLSLExtension.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("gl-shader");

namespace fs = std::filesystem;

GLSLExtension* Shader::preprocessor = new GLSLExtension();
Shader* Shader::used = nullptr;

Shader::Shader(uint id, Source&& vertexSource, Source&& fragmentSource)
    : id(id),
      vertexSource(std::move(vertexSource)),
      fragmentSource(std::move(fragmentSource)) {}

Shader::~Shader() {
    glDeleteProgram(id);
}

void Shader::use() {
    used = this;
    glUseProgram(id);
}

uint Shader::getUniformLocation(const std::string& name) {
    auto found = uniformLocations.find(name);
    if (found == uniformLocations.end()) {
        uint location = glGetUniformLocation(id, name.c_str());
        uniformLocations.try_emplace(name, location);
        return location;
    }
    return found->second;
}

void Shader::uniformMatrix(const std::string& name, const glm::mat4& matrix) {
    glUniformMatrix4fv(
        getUniformLocation(name), 1, GL_FALSE, glm::value_ptr(matrix)
    );
}

void Shader::uniformMatrix(const std::string& name, const glm::mat3& matrix) {
    glUniformMatrix3fv(
        getUniformLocation(name), 1, GL_FALSE, glm::value_ptr(matrix)
    );
}

void Shader::uniform1i(const std::string& name, int x){
    glUniform1i(getUniformLocation(name), x);
}

void Shader::uniform1f(const std::string& name, float x){
    glUniform1f(getUniformLocation(name), x);
}

void Shader::uniform2f(const std::string& name, float x, float y){
    glUniform2f(getUniformLocation(name), x, y);
}

void Shader::uniform2f(const std::string& name, const glm::vec2& xy){
    glUniform2f(getUniformLocation(name), xy.x, xy.y);
}

void Shader::uniform2i(const std::string& name, const glm::ivec2& xy){
    glUniform2i(getUniformLocation(name), xy.x, xy.y);
}

void Shader::uniform3f(const std::string& name, float x, float y, float z){
    glUniform3f(getUniformLocation(name), x,y,z);
}

void Shader::uniform3f(const std::string& name, const glm::vec3& xyz){
    glUniform3f(getUniformLocation(name), xyz.x, xyz.y, xyz.z);
}

void Shader::uniform4f(const std::string& name, const glm::vec4& xyzw) {
    glUniform4f(getUniformLocation(name), xyzw.x, xyzw.y, xyzw.z, xyzw.w);
}

void Shader::uniform1v(const std::string& name, int length, const int* v) {
    glUniform1iv(getUniformLocation(name), length, v);
}

void Shader::uniform1v(const std::string& name, int length, const float* v) {
    glUniform1fv(getUniformLocation(name), length, v);
}

void Shader::uniform2v(const std::string& name, int length, const float* v) {
    glUniform2fv(getUniformLocation(name), length, v);
}

void Shader::uniform3v(const std::string& name, int length, const float* v) {
    glUniform3fv(getUniformLocation(name), length, v);
}

void Shader::uniform4v(const std::string& name, int length, const float* v) {
    glUniform4fv(getUniformLocation(name), length, v);
}

static inline auto shader_deleter = [](GLuint* shader) {
    glDeleteShader(*shader);
    delete shader;
};

inline const uint GL_LOG_LEN = 512;

// shader should be deleted after shader program linking
using glshader = std::unique_ptr<GLuint, decltype(shader_deleter)>;

glshader compile_shader(GLenum type, const GLchar* source, const std::string& file) {
    GLint success;
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[GL_LOG_LEN];
        glGetShaderInfoLog(shader, GL_LOG_LEN, nullptr, infoLog);
        glDeleteShader(shader);
        throw std::runtime_error(
            "vertex shader compilation failed (" + file + "):\n" +
            std::string(infoLog)
        );
    }
    return glshader(new GLuint(shader), shader_deleter); //-V508
}

static GLuint compile_program(
    const Shader::Source& vertexSource, const Shader::Source& fragmentSource
) {
    auto& preprocessor = *Shader::preprocessor;

    auto vertexCode = std::move(
        preprocessor.process(vertexSource.file, vertexSource.code).code
    );
    auto fragmentCode = std::move(
        preprocessor.process(fragmentSource.file, fragmentSource.code).code
    );

    const GLchar* vCode = vertexCode.c_str();
    const GLchar* fCode = fragmentCode.c_str();

    glshader vertex =
        compile_shader(GL_VERTEX_SHADER, vCode, vertexSource.file);
    glshader fragment =
        compile_shader(GL_FRAGMENT_SHADER, fCode, fragmentSource.file);

    // Shader Program
    GLint success;
    GLuint program = glCreateProgram();
    glAttachShader(program, *vertex);
    glAttachShader(program, *fragment);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success) {
        GLchar infoLog[GL_LOG_LEN];
        glGetProgramInfoLog(program, GL_LOG_LEN, nullptr, infoLog);
        throw std::runtime_error(
            "shader program linking failed:\n" + std::string(infoLog)
        );
    }
    return program;
}

void Shader::recompile() {
    GLuint newProgram = compile_program(vertexSource, fragmentSource);
    glDeleteProgram(id);
    id = newProgram;
    uniformLocations.clear();
    logger.info() << "shader " << id << " has been recompiled";
}

std::unique_ptr<Shader> Shader::create(
    Source&& vertexSource, Source&& fragmentSource
) {
    return std::make_unique<Shader>(
        compile_program(vertexSource, fragmentSource),
        std::move(vertexSource),
        std::move(fragmentSource)
    );
}

Shader& Shader::getUsed() {
    return *used;
}
