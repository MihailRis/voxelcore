#ifndef FOG_GLSL_
#define FOG_GLSL_

uniform float u_fogFactor;
uniform float u_fogCurve;
uniform float u_weatherFogOpacity;
uniform float u_weatherFogDencity;
uniform float u_weatherFogCurve;

float calc_fog(float depth, float fogFactor, float fogCurve) {
    return min(
        1.0,
        max(pow(depth * u_fogFactor * fogFactor, u_fogCurve * fogCurve),
            min(pow(depth * u_weatherFogDencity, u_weatherFogCurve),
                u_weatherFogOpacity))
    );
}

float calc_fog(float depth) {
    return calc_fog(depth, 1.0, 1.0);
}

#endif // FOG_GLSL_
