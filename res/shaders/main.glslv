#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec4 v_light;
layout (location = 3) in vec4 v_normal;

#include <world_vertex_header>
#include <lighting>
#include <fog>
#include <sky>

uniform float u_dayTime;

out vec4 a_torchLight;

#include <__materials_vertex__>

void main() {
    a_material = unpack_material(v_normal.w);
    a_realnormal = v_normal.xyz * 2.0 - 1.0;
    a_modelpos = u_model * vec4(v_position, 1.0f);
    // material shader may also modify a_realnormal
    a_modelpos.xyz = apply_material_vertex(a_material, a_modelpos.xyz);
    vec3 pos3d = a_modelpos.xyz - u_cameraPos;

    a_normal = calc_screen_normal(a_realnormal);

    a_torchLight = vec4(calc_torch_light(
        v_light.rgb, a_realnormal, a_modelpos.xyz, u_torchlightColor, u_gamma
    ), 1.0);
    a_texCoord = v_texCoord;

    a_dir = a_modelpos.xyz - u_cameraPos;
    vec3 skyLightColor = pick_sky_color(u_skybox, u_dayTime, u_minSkyLight);
    a_skyLight = skyLightColor.rgb*v_light.a;

    mat4 viewmodel = u_view * u_model;
    a_distance = length(viewmodel * vec4(pos3d, 0.0));

#ifndef ADVANCED_RENDER
    a_fog = calc_fog(length(viewmodel * vec4(pos3d * FOG_POS_SCALE, 0.0)) / 256.0);
#endif

    a_emission = unpack_emission(v_normal.w);

    vec4 viewmodelpos = u_view * a_modelpos;
    a_position = viewmodelpos.xyz;
    gl_Position = u_proj * viewmodelpos;
}
