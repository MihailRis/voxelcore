#ifndef GLSL_WORLD_UNIFORMS_
#define GLSL_WORLD_UNIFORMS_

uniform mat4 u_model;
uniform mat4 u_proj;
uniform mat4 u_view;
uniform vec3 u_cameraPos;
uniform float u_gamma;
uniform float u_opacity;
uniform float u_timer;
uniform samplerCube u_skybox;
uniform vec3 u_torchlightColor;
uniform float u_torchlightDistance;

#endif // GLSL_WORLD_UNIFORMS_
