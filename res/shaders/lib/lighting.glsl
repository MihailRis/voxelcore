#ifndef LIGHTING_GLSL_
#define LIGHTING_GLSL_

uniform float u_fogFactor;
uniform float u_fogCurve;
uniform float u_weatherFogOpacity;
uniform float u_weatherFogDencity;
uniform float u_weatherFogCurve;

float calc_torch_light(vec3 modelpos) {
    return max(0.0, 1.0 - distance(u_cameraPos, modelpos.xyz) / u_torchlightDistance);
}

vec3 calc_screen_normal(vec3 normal) {
    return transpose(inverse(mat3(u_view * u_model))) * normal;
}

float calc_fog(float depth) {
    return min(
        1.0,
        max(pow(depth * u_fogFactor, u_fogCurve),
            min(pow(depth * u_weatherFogDencity, u_weatherFogCurve),
                u_weatherFogOpacity))
    );
}

#endif // LIGHTING_GLSL_
