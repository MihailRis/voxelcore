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
uniform bool u_alphaClip;

const float BAYER_MATRIX[64] = float[](
     0.0, 32.0,  8.0, 40.0,  2.0, 34.0, 10.0, 42.0,
    48.0, 16.0, 56.0, 24.0, 50.0, 18.0, 58.0, 26.0,
    12.0, 44.0,  4.0, 36.0, 14.0, 46.0,  6.0, 38.0,
    60.0, 28.0, 52.0, 20.0, 62.0, 30.0, 54.0, 22.0,
     3.0, 35.0, 11.0, 43.0,  1.0, 33.0,  9.0, 41.0,
    51.0, 19.0, 59.0, 27.0, 49.0, 17.0, 57.0, 25.0,
    15.0, 47.0,  7.0, 39.0, 13.0, 45.0,  5.0, 37.0,
    63.0, 31.0, 55.0, 23.0, 61.0, 29.0, 53.0, 21.0
);

float calc_bayer(vec2 pos){
    ivec2 p = ivec2(mod(pos, 8.0));
    return BAYER_MATRIX[p.y * 8 + p.x] / 64.0;
}

void main() {
    vec4 texColor = texture(u_texture0, a_texCoord);
    float alpha = a_color.a * texColor.a;
    if (u_opaque) {
        float bayer = calc_bayer(gl_FragCoord.xy);
        if (alpha - bayer < 0.01) {
            discard;
        }
        alpha = 1.0;
    } else {
        if (alpha < u_alphaThreshold) {
            discard;
        }
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
