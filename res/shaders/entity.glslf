in float a_distance;
in vec4 a_color;
in vec2 a_texCoord;
in vec3 a_position;
in vec3 a_dir;
in vec3 a_normal;
in vec3 a_realnormal;
in vec4 a_modelpos;
in float a_fog;

layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;

uniform sampler2D u_texture0;
uniform samplerCube u_skybox;
uniform vec3 u_fogColor;
uniform float u_fogFactor;
uniform float u_fogCurve;
uniform bool u_alphaClip;
uniform vec3 u_sunDir;

uniform bool u_enableShadows;

#include <shadows>

void main() {
    float shadow = calc_shadow();
    vec3 fogColor = texture(u_skybox, a_dir).rgb;
    vec4 tex_color = texture(u_texture0, a_texCoord);
    float alpha = a_color.a * tex_color.a;
    // anyway it's any alpha-test alternative required
    if (alpha < (u_alphaClip ? 0.5f : 0.15f)) {
        discard;
    }
    f_color = a_color * tex_color;
    f_color.rgb *= shadow;
    f_color = mix(f_color, vec4(fogColor, 1.0), a_fog);
    f_color.a = alpha;
    f_position = vec4(a_position, 1.0);
    f_normal = vec4(a_normal, 1.0);
}
