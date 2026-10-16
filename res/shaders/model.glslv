#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec3 v_normal;
layout (location = 3) in mat4 i_model;
layout (location = 7) in vec4 i_tint;
layout (location = 8) in vec4 i_light;
layout (location = 9) in vec4 i_uvRegion;
layout (location = 10) in vec4 i_params;

#include <world_vertex_header>
#include <lighting>
#include <fog>
#include <sky>

out vec4 a_color;

uniform float u_dayTime;

const vec3 SUN_VECTOR = vec3(0.411934, 0.863868, -0.279161);

void main() {
    a_modelpos = i_model * vec4(v_position, 1.0);
    vec3 pos3d = a_modelpos.xyz - u_cameraPos;

    vec3 normal = mat3(i_model) * v_normal;
    float normalLength = length(normal);
    normal = normalLength > 1e-6 ? normal / normalLength : v_normal;

    bool shading = i_params.x > 0.5;
    vec4 light = i_light;
    if (shading) {
        light *= 0.8 + dot(normal, SUN_VECTOR) * 0.2;
    }

    a_realnormal = normal;
    a_normal = calc_screen_normal(a_realnormal);

    a_color = vec4(calc_torch_light(
        light.rgb, a_realnormal, a_modelpos.xyz, u_torchlightColor, u_gamma
    ), 1.0);
    a_texCoord = v_texCoord * i_uvRegion.zw + i_uvRegion.xy;

    a_dir = a_modelpos.xyz - u_cameraPos;
    vec3 skyLightColor = pick_sky_color(u_skybox, u_dayTime, u_minSkyLight);
    a_color.rgb = max(a_color.rgb, skyLightColor.rgb * light.a) * i_tint.rgb;
    a_color.a = u_opacity * i_tint.a;

    mat4 viewmodel = u_view * u_model;
    a_distance = length(viewmodel * vec4(pos3d, 0.0));
    a_fog = calc_fog(length(viewmodel * vec4(pos3d * FOG_POS_SCALE, 0.0)) / 256.0);
    a_emission = shading ? 0.0 : 1.0;

    vec4 viewmodelpos = u_view * a_modelpos;
    a_position = viewmodelpos.xyz;
    gl_Position = u_proj * viewmodelpos;
}
