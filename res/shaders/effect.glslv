layout (location = 0) in vec2 v_position;

out vec2 v_uv;

uniform ivec2 u_screenSize;

void main(){
    v_uv = v_position * 0.5 + 0.5;
    gl_Position = vec4(v_position, 0.0, 1.0);
}
