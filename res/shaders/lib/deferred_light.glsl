#ifndef DEFERRED_LIGHT_GLSL_
#define DEFERRED_LIGHT_GLSL_

#include <shadows>

#define COLOR_TEMPERATURE_COMPENSATION 1.14f

vec3 calc_deferred_light(
    vec4 modelpos, vec3 normal, float distance, float emission, float ao
) {
    float light = ao;
    float rg = 1.0;
#ifdef ENABLE_SHADOWS
    float shadow_mul = calc_shadow(modelpos, normal, distance);
    light *= shadow_mul;
    rg *= min(1.0, shadow_mul + emission) * COLOR_TEMPERATURE_COMPENSATION;
    light *= 2.0;
#endif
    light = max(light, emission);
    light = pow(light, u_gamma);
    return light * vec3(rg, 0.5 + rg * 0.5, 1.0);
}

#endif // DEFERRED_LIGHT_GLSL_
