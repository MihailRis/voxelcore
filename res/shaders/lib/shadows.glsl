#ifndef SHADOWS_GLSL_
#define SHADOWS_GLSL_

#include <constants>

uniform sampler2DShadow u_shadows[2];
uniform mat4 u_shadowsMatrix[2];
uniform float u_dayTime;
uniform int u_shadowsRes;
uniform float u_shadowsOpacity;
uniform float u_shadowsSoftness;

float calc_shadow(
    sampler2DShadow shadowsMap, 
    mat4 shadowMatrix, 
    vec4 modelPos, 
    vec3 realnormal, 
    vec3 normalOffset, 
    float bias
) {
    float step = 1.0 / float(u_shadowsRes);
    vec4 mpos = shadowMatrix * vec4(modelPos.xyz + normalOffset, 1.0);
    vec3 projCoords = mpos.xyz / mpos.w;
    projCoords = projCoords * 0.5 + 0.5;
    projCoords.z -= 0.00001 / u_shadowsRes + bias;

    float shadow = 0.0;
    if (dot(realnormal, u_sunDir) < 0.0) {
        // 3x3 kernel
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                vec3 offset = vec3(x, y, -(abs(x) + abs(y)) * 0.8) * step * 1.0 * u_shadowsSoftness;
                shadow += texture(shadowsMap, projCoords + offset);
            }
        }
        shadow /= 9.0;
    } else {
        shadow = 0.0;
    }
    return shadow;
}

// TODO: add array textures support
float calc_shadow(vec4 modelPos, vec3 realnormal, float distance) {
#ifdef ENABLE_SHADOWS
    float s = pow(abs(cos(u_dayTime * PI2)), 0.25) * u_shadowsOpacity;
    vec3 normalOffset = realnormal * (distance > 64.0 ? 0.2 : 0.04);

    // as slow as mix(...) 
    float shadow = (distance < 80.0) 
        ? calc_shadow(u_shadows[0], u_shadowsMatrix[0], modelPos, realnormal, normalOffset, 0.0)
        : calc_shadow(u_shadows[1], u_shadowsMatrix[1], modelPos, realnormal, normalOffset, 0.001);
    
    return 0.5 * (1.0 + s * shadow);
#else
    return 1.0;
#endif
}

#endif // SHADOWS_GLSL_
