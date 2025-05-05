in vec3 v_coord;
layout (location = 0) out vec4 f_color;
layout (location = 1) out vec3 f_position;
layout (location = 2) out vec3 f_normal;

uniform samplerCube u_cubemap;

void main(){
    vec3 dir = normalize(v_coord);
    f_color = texture(u_cubemap, dir);
}
