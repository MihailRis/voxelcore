#pragma once

#include <string>

#include "data/dv_fwd.hpp"

class Content;
class Shader;
class ResPaths;
class GLSLExtension;

/// @brief Material shaders (shaders/materials/<name>.glslv, .glslf) are
/// included into the world shaders (main, translucent, shadows, entity)
namespace material_shaders {
    inline constexpr const char* VERTEX_HEADER = "__materials_vertex__";
    inline constexpr const char* FRAGMENT_HEADER = "__materials_fragment__";

    /// @brief Generate the world shaders headers from the material shaders.
    /// Must be called before the world shaders (re)compilation.
    /// @param content loaded content (nullable)
    void build_headers(
        GLSLExtension& preprocessor,
        const ResPaths& paths,
        const Content* content
    );

    /// @brief Upload the material shaders params to the shader
    /// if changed since the previous call for this shader program
    void setup_shader(Shader& shader);

    /// @brief Set the material shader params values ('param' directives)
    /// @param material material name (pack:name)
    /// @throws std::runtime_error if material or param does not exist
    void set_params(const std::string& material, const dv::value& params);

    /// @brief Get the material shader params current values
    /// @return table of param name -> value or null if material has no shader
    dv::value get_params(const std::string& material);
}
