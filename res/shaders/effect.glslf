in vec2 v_uv;
out vec4 f_color;

uniform sampler2D u_screen;
uniform ivec2 u_screenSize;
uniform float u_intensity;

#include <__effect__>

void main() {
    f_color = effect();
}

