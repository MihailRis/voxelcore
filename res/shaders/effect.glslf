in vec2 v_uv;
out vec4 f_color;

uniform sampler2D u_screen;
uniform sampler2D u_position;
uniform sampler2D u_normal;
uniform sampler2D u_noise;
uniform sampler2D u_ssao;
uniform sampler2D u_shadows;

uniform ivec2 u_screenSize;
uniform float u_intensity;
uniform float u_timer;
uniform bool u_enableShadows;
uniform mat4 u_projection;

#include <__effect__>

void main() {
    f_color = effect();
}

