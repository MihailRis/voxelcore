#ifndef FOG_GLSL_
#define FOG_GLSL_

uniform float u_fogFactor;
uniform float u_fogCurve;
uniform float u_weatherFogOpacity;
uniform float u_weatherFogDencity;
uniform float u_weatherFogCurve;

float calc_fog(float depth) {
    return min(
        1.0,
        max(pow(depth * u_fogFactor, u_fogCurve),
            min(pow(depth * u_weatherFogDencity, u_weatherFogCurve),
                u_weatherFogOpacity))
    );
}

#endif // FOG_GLSL_
