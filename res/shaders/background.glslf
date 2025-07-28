in vec3 v_coord;
layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;

uniform mat4 u_view;
uniform samplerCube u_skybox;

void main(){
    vec3 dir = normalize(v_coord) * 1e6;
    f_position = u_view * vec4(dir, 1.0);
    f_normal = vec4(0.0, 0.0, 1.0, 1.0);
    f_color = texture(u_skybox, dir);
}
