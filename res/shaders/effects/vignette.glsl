#include <commons>

#param float p_radius = 1.1
#param float p_softness = 0.7

vec4 apply_vignette(vec4 color) {
    vec2 position = v_uv - vec2(0.5);
    float dist = length(position);
    float vignette = smoothstep(p_radius, p_radius - p_softness, dist);
    return vec4(color.rgb * vignette, 1.0);
}

vec4 effect() {
    return apply_vignette(texture(u_screen, v_uv));
}
