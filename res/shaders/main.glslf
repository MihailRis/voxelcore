layout (location = 0) out vec4 f_color;
layout (location = 1) out vec4 f_position;
layout (location = 2) out vec4 f_normal;

in vec4 a_color;
in vec2 a_texCoord;
in float a_fog;
in vec3 a_position;
in vec3 a_dir;
in vec3 a_normal;

uniform sampler2D u_texture0;
uniform samplerCube u_cubemap;
uniform bool u_alphaClip;
uniform bool u_debugLights;
uniform bool u_debugNormals;
uniform sampler2D u_shadows;

void main() {
    vec3 fogColor = texture(u_cubemap, a_dir).rgb;
    vec4 tex_color = texture(u_texture0, a_texCoord);
    float alpha = a_color.a * tex_color.a;
    if (u_alphaClip) {
        if (alpha < 0.2f)
            discard;
        alpha = 1.0;
    } else {
        if (alpha < 0.002f)
            discard;
    }
    if (u_debugLights)
        tex_color.rgb = u_debugNormals ? (a_normal * 0.5 + 0.5) : vec3(1.0);
    else if (u_debugNormals) {
        tex_color.rgb *= a_normal * 0.5 + 0.5;
    }
    f_color = mix(a_color * tex_color, vec4(fogColor,1.0), a_fog);
    f_color.a = alpha;
    f_position = vec4(a_position, 1.0);
    f_normal = vec4(a_normal, 1.0);
}
