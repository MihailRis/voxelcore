uniform vec3 samples[64];

vec4 effect() {
    vec2 noiseScale = u_screenSize / 4.0; 

    vec3 position = texture(u_position, v_uv).xyz;
    vec3 color = texture(u_screen, v_uv).rgb;
    vec3 normal = texture(u_normal, v_uv).xyz;
    vec3 randomVec = texture(u_noise, v_uv * noiseScale).xyz;
    return vec4(randomVec, 1.0);
}
