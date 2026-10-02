#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec4 v_light;
layout (location = 3) in vec4 v_normal;

// same vertex stage interface as world shaders for material shaders
#include <world_vertex_header>

uniform float u_dayTime;

#include <__materials_vertex__>

void main() {
    a_texCoord = v_texCoord;
    a_material = unpack_material(v_normal.w);
    a_realnormal = v_normal.xyz * 2.0 - 1.0;
    a_modelpos = u_model * vec4(v_position, 1.0f);
    a_modelpos.xyz = apply_material_vertex(a_material, a_modelpos.xyz);
    gl_Position = u_proj * u_view * a_modelpos;
}
