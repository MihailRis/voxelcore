#ifndef COMMONS_GLSL_
#define COMMONS_GLSL_

#include <constants>

vec3 apply_planet_curvature(vec3 modelPos, vec3 pos3d) {
    modelPos.y -= pow(length(pos3d.xz) * CURVATURE_FACTOR, 3.0f);
    return modelPos;
}

// chunk vertex flags (normal.w): bit 7 - emission flag, bits 0-6 - material
float unpack_emission(float w) {
    return float(int(w * 255.0 + 0.5) >> 7);
}

int unpack_material(float w) {
    return int(w * 255.0 + 0.5) & 127;
}

#endif // COMMONS_GLSL_
