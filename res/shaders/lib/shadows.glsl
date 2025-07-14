#ifndef SHADOWS_GLSL_
#define SHADOWS_GLSL_

#include <constants>

uniform sampler2DShadow u_shadows[2];
uniform mat4 u_shadowsMatrix[2];
uniform float u_dayTime;
uniform int u_shadowsRes;
uniform float u_shadowsOpacity;
uniform float u_shadowsSoftness;

float calc_shadow(vec4 modelPos, vec3 realnormal, float distance) {
#ifdef ENABLE_SHADOWS
    float step = 1.0 / float(u_shadowsRes);
    float s = pow(abs(cos(u_dayTime * PI2)), 0.25) * u_shadowsOpacity;
    vec3 normalOffset = realnormal * (distance > 64.0 ? 0.2 : 0.04);
    int shadowIdx = distance > 80.0 ? 1 : 0;

    vec4 mpos = u_shadowsMatrix[shadowIdx] * vec4(modelPos.xyz + normalOffset, 1.0);
    vec3 projCoords = mpos.xyz / mpos.w;
    projCoords = projCoords * 0.5 + 0.5;
    projCoords.z -= 0.00001 / u_shadowsRes;
    if (shadowIdx > 0) {
        projCoords.z -= 0.001;
    }

    float shadow = 0.0;
    if (dot(realnormal, u_sunDir) < 0.0) {
        // 3x3 kernel
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                vec3 offset = vec3(x, y, -(abs(x) + abs(y)) * 0.8) * step * 1.0 * u_shadowsSoftness;
                shadow += texture(u_shadows[shadowIdx], projCoords + offset);
            }
        }
        shadow /= 9.0;
    } else {
        shadow = 0.5;
    }
    return 0.5 * (1.0 + s * shadow);
#else
    return 1.0;
#endif
}

#endif // SHADOWS_GLSL_
