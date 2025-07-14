#ifndef LIGHTING_GLSL_
#define LIGHTING_GLSL_

float calc_torch_light(vec3 normal, vec3 modelpos) {
    return max(0.0, 1.0 - distance(u_cameraPos, modelpos) / u_torchlightDistance)
         * max(0.0, -dot(normal, normalize(modelpos - u_cameraPos)));
}

vec3 calc_screen_normal(vec3 normal) {
    return transpose(inverse(mat3(u_view * u_model))) * normal;
}

#endif // LIGHTING_GLSL_
