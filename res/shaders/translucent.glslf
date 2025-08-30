layout (location = 0) out vec4 f_color;

#include <world_fragment_header>

in vec4 a_torchLight;

uniform sampler2D u_texture0;
uniform vec3 u_sunDir;

// flags
uniform bool u_alphaClip;
uniform bool u_debugLights;
uniform bool u_debugNormals;

#include <shadows>

void main() {
    vec4 texColor = texture(u_texture0, a_texCoord);
    float alpha = texColor.a;
    if (u_alphaClip) {
        if (alpha < 0.2f)
            discard;
        alpha = 1.0;
    } else {
        if (alpha < 0.002f)
            discard;
    }
    if (u_debugLights)
        texColor.rgb = u_debugNormals ? (a_normal * 0.5 + 0.5) : vec3(1.0);

    f_color = texColor;
    f_color.rgb *= min(vec3(1.0), a_torchLight.rgb + a_skyLight
        * calc_shadow(a_modelpos, a_realnormal, length(a_position)));

    vec3 fogColor = texture(u_skybox, a_dir).rgb;
    f_color = mix(f_color, vec4(fogColor, 1.0), a_fog);
    f_color.a = alpha;
}
