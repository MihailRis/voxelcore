#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec4 v_light;
layout (location = 3) in vec4 v_normal;

out float a_distance;
out float a_fog;
out vec2 a_texCoord;
out vec3 a_dir;
out vec3 a_normal;
out vec3 a_position;
out vec3 a_realnormal;
out vec4 a_torchLight;
out vec3 a_skyLight;
out vec4 a_modelpos;
out float a_emission;

uniform mat4 u_model;
uniform mat4 u_proj;
uniform mat4 u_view;
uniform vec3 u_cameraPos;
uniform float u_gamma;
uniform float u_timer;
uniform samplerCube u_skybox;

uniform vec3 u_torchlightColor;
uniform float u_torchlightDistance;

#include <lighting>
#include <fog>

void main() {
    a_modelpos = u_model * vec4(v_position, 1.0f);
    vec3 pos3d = a_modelpos.xyz - u_cameraPos;
    a_modelpos.xyz = apply_planet_curvature(a_modelpos.xyz, pos3d);

    a_realnormal = v_normal.xyz * 2.0 - 1.0;
    a_normal = calc_screen_normal(a_realnormal);

    vec3 light = v_light.rgb;
    float torchlight = calc_torch_light(a_realnormal, a_modelpos.xyz);
    a_torchLight = vec4(pow(light + torchlight * u_torchlightColor, vec3(u_gamma)), 1.0f);

    a_texCoord = v_texCoord;

    a_dir = a_modelpos.xyz - u_cameraPos;
    vec3 skyLightColor = pick_sky_color(u_skybox);
    a_skyLight = skyLightColor.rgb*v_light.a;

    mat4 viewmodel = u_view * u_model;
    a_distance = length(viewmodel * vec4(pos3d, 0.0));
#ifndef ADVANCED_RENDER
    a_fog = calc_fog(length(viewmodel * vec4(pos3d * FOG_POS_SCALE, 0.0)) / 256.0);
#endif
    a_emission = v_normal.w;

    vec4 viewmodelpos = u_view * a_modelpos;
    a_position = viewmodelpos.xyz;
    gl_Position = u_proj * viewmodelpos;
}
