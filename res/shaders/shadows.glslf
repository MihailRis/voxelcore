in vec2 a_texCoord;

uniform sampler2D u_texture0;

void main() {
    vec4 tex_color = texture(u_texture0, a_texCoord);
    if (tex_color.a < 0.5) {
        discard;
    }
    // depth will be written anyway
}
