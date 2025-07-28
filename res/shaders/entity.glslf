layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;
layout (location = 3) out vec4 f_emission;

#include <world_fragment_header>

in vec4 a_color;

uniform sampler2D u_texture0;

uniform vec3 u_fogColor;
uniform float u_fogFactor;
uniform float u_fogCurve;
uniform bool u_alphaClip;
uniform vec3 u_sunDir;

void main() {
    vec4 texColor = texture(u_texture0, a_texCoord);
    float alpha = a_color.a * texColor.a;
    // anyway it's any alpha-test alternative required
    if (alpha < (u_alphaClip ? 0.5f : 0.15f)) {
        discard;
    }
    f_color = a_color * texColor;

#ifndef ADVANCED_RENDER
    vec3 fogColor = texture(u_skybox, a_dir).rgb;
    f_color = mix(f_color, vec4(fogColor, 1.0), a_fog);
#endif

    f_color.a = alpha;
    f_position = vec4(a_position, 1.0);
    f_normal = vec4(a_normal, 1.0);
    f_emission = vec4(vec3(a_emission), 1.0);
}
