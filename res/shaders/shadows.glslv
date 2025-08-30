#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec4 v_light;
layout (location = 3) in vec4 v_normal;

out vec2 a_texCoord;

uniform mat4 u_model;
uniform mat4 u_proj;
uniform mat4 u_view;

void main() {
    a_texCoord = v_texCoord;
    gl_Position = u_proj * u_view * u_model * vec4(v_position, 1.0f);
}
