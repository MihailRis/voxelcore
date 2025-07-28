#ifndef COMMONS_GLSL_
#define COMMONS_GLSL_

#include <constants>

vec3 apply_planet_curvature(vec3 modelPos, vec3 pos3d) {
    modelPos.y -= pow(length(pos3d.xz) * CURVATURE_FACTOR, 3.0f);
    return modelPos;
}

#endif // COMMONS_GLSL_
