#ifndef COMMONS_GLSL_
#define COMMONS_GLSL_
#include <constants>

vec3 pick_sky_color(samplerCube cubemap) {
    vec3 skyLightColor = texture(cubemap, vec3(0.4f, 0.0f, 0.4f)).rgb;
    skyLightColor *= SKY_LIGHT_TINT;
    skyLightColor = min(vec3(1.0f), skyLightColor * SKY_LIGHT_MUL);
    skyLightColor = max(MIN_SKY_LIGHT, skyLightColor);
    return skyLightColor;
}

vec3 apply_planet_curvature(vec3 modelPos, vec3 pos3d) {
    modelPos.y -= pow(length(pos3d.xz) * CURVATURE_FACTOR, 3.0f);
    return modelPos;
}

#endif // COMMONS_GLSL_
