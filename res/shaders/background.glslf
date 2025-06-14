in vec3 v_coord;
layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;

uniform samplerCube u_skybox;

void main(){
    vec3 dir = normalize(v_coord);
    f_position = vec4(0.0, 0.0, 0.0, 0.0);
    f_normal = vec4(0.0);
    f_color = texture(u_skybox, dir);
}
