#ifndef GLSL_WORLD_FRAGMENT_HEADER_
#define GLSL_WORLD_FRAGMENT_HEADER_

in float a_distance;
in float a_fog;
in vec2 a_texCoord;
in vec3 a_dir;
in vec3 a_normal;
in vec3 a_position;
in vec3 a_realnormal;
in vec3 a_skyLight;
in vec4 a_modelpos;
in float a_emission;

#include <world_uniforms>

#endif // GLSL_WORLD_FRAGMENT_HEADER_
