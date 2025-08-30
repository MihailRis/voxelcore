layout (location = 0) in vec2 v_position;

out vec3 v_coord;

uniform mat4 u_view;
uniform float u_ar;
uniform float u_zoom;

void main(){
    v_coord = (vec4(v_position * vec2(u_ar, 1.0f) * u_zoom, -1.0, 1.0) * u_view).xyz;
    gl_Position = vec4(v_position, 1.0 - 1e-6, 1.0);
}
