layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;
layout (location = 3) out vec4 f_emission;

uniform vec3 u_sunDir;

#include <world_fragment_header>
#include <deferred_light>

in vec4 a_color;

uniform sampler2D u_texture0;
uniform float u_alphaThreshold;
uniform bool u_opaque;
uniform bool u_forward;
uniform bool u_forwardLight;

void main() {
    vec4 texColor = texture(u_texture0, a_texCoord);
    float alpha = a_color.a * texColor.a;
    if (alpha < u_alphaThreshold) {
        discard;
    }
    f_color = a_color * texColor;

#ifdef ADVANCED_RENDER
    if (u_forward) {
        if (u_forwardLight) {
            f_color.rgb *= calc_deferred_light(
                a_modelpos, a_realnormal, length(a_position), a_emission, 1.0
            );
        }
        vec3 fogColor = texture(u_skybox, a_dir).rgb;
        f_color.rgb = mix(f_color.rgb, fogColor, a_fog);
    }
#else
    vec3 fogColor = texture(u_skybox, a_dir).rgb;
    f_color.rgb = mix(f_color.rgb, fogColor, a_fog);
#endif

    f_color.a = u_opaque ? 1.0 : alpha;
    f_position = vec4(a_position, 1.0);
    f_normal = vec4(a_normal, 1.0);
    f_emission = vec4(vec3(a_emission), 1.0);
}
