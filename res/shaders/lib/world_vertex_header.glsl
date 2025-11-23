#ifndef GLSL_WORLD_VERTEX_HEADER_
#define GLSL_WORLD_VERTEX_HEADER_

out float a_distance;
out float a_fog;
out vec2 a_texCoord;
out vec3 a_dir;
out vec3 a_normal;
out vec3 a_position;
out vec3 a_realnormal;
out vec3 a_skyLight;
out vec4 a_modelpos;
out float a_emission;

#include <world_uniforms>

#endif // GLSL_WORLD_VERTEX_HEADER_
