#include <world_fragment_header>

uniform sampler2D u_texture0;

#include <__materials_fragment__>

void main() {
    vec4 tex_color = apply_material_fragment(
        a_material, texture(u_texture0, a_texCoord)
    );
    if (tex_color.a < 0.5) {
        discard;
    }
    // depth will be written anyway
}
