in vec4 a_color;
in vec2 a_texCoord;
in float a_fog;
in vec3 a_dir;
out vec4 f_color;

uniform sampler2D u_texture0;
uniform samplerCube u_cubemap;
uniform bool u_alphaClip;
uniform bool u_debugLights;

void main() {
    vec3 fogColor = texture(u_cubemap, a_dir).rgb;
    vec4 tex_color = texture(u_texture0, a_texCoord);
    if (u_debugLights)
        tex_color.rgb = vec3(1.0);
    float alpha = a_color.a * tex_color.a;
    if (u_alphaClip) {
        if (alpha < 0.2f)
            discard;
        alpha = 1.0;
    } else {
        if (alpha < 0.002f)
            discard;
    }
    f_color = mix(a_color * tex_color, vec4(fogColor,1.0), a_fog);
    f_color.a = alpha;
}
