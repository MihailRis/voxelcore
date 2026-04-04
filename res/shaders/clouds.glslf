layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;
layout (location = 3) out vec4 f_emission;

#include <world_fragment_header>

in vec4 a_torchLight;

uniform sampler2D u_texture0;
uniform vec3 u_sunDir;

// flags
uniform bool u_alphaClip;
uniform bool u_debugLights;
uniform bool u_debugNormals;
uniform vec4 u_tint;

void main() {
    f_color.rgb = max(a_torchLight.rgb, a_skyLight);
    f_color.a = 1.0;
    f_color *= u_tint;

#ifndef ADVANCED_RENDER
    vec3 fogColor = texture(u_skybox, a_dir).rgb;
    f_color = mix(f_color, vec4(fogColor, 1.0), a_fog);
#endif
    f_position = vec4(a_position, 1.0);
    f_normal = vec4(a_normal, 1.0);
    f_emission = vec4(vec3(a_emission), 1.0);
}
