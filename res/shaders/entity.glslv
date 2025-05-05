#include <commons>

layout (location = 0) in vec3 v_position;
layout (location = 1) in vec2 v_texCoord;
layout (location = 2) in vec3 v_color;
layout (location = 3) in vec4 v_light;

out vec4 a_color;
out vec2 a_texCoord;
out vec3 a_normal;
out float a_fog;
out vec3 a_position;
out vec3 a_dir;

uniform mat4 u_model;
uniform mat4 u_proj;
uniform mat4 u_view;
uniform vec3 u_cameraPos;
uniform float u_gamma;
uniform float u_opacity;
uniform float u_fogFactor;
uniform float u_fogCurve;
uniform float u_weatherFogOpacity;
uniform float u_weatherFogDencity;
uniform float u_weatherFogCurve;
uniform samplerCube u_cubemap;

uniform vec3 u_torchlightColor;
uniform float u_torchlightDistance;

void main() {
    vec4 modelpos = u_model * vec4(v_position, 1.0);
    vec3 pos3d = modelpos.xyz - u_cameraPos;
    modelpos.xyz = apply_planet_curvature(modelpos.xyz, pos3d);

    a_normal = vec3(0.0, 1.0, 0.0);//v_normal.xyz * 2.0 - 1.0;

    vec3 light = v_light.rgb;
    float torchlight = max(0.0, 1.0-distance(u_cameraPos, modelpos.xyz) / 
                       u_torchlightDistance);
    light += torchlight * u_torchlightColor;
    a_color = vec4(pow(light, vec3(u_gamma)),1.0f);
    a_texCoord = v_texCoord;

    a_dir = modelpos.xyz - u_cameraPos;
    vec3 skyLightColor = pick_sky_color(u_cubemap);
    a_color.rgb = max(a_color.rgb, skyLightColor.rgb*v_light.a) * v_color;
    a_color.a = u_opacity;

    float dist = length(u_view * u_model * vec4(pos3d * FOG_POS_SCALE, 0.0));
    float depth = (dist / 256.0);
    a_fog = min(1.0, max(pow(depth * u_fogFactor, u_fogCurve),
                         min(pow(depth * u_weatherFogDencity, u_weatherFogCurve), u_weatherFogOpacity)));
    gl_Position = u_proj * u_view * modelpos;

    a_position = (u_view * modelpos).xyz;
}
