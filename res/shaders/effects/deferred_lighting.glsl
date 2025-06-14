vec4 effect() {
    float ssao = 0.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            vec2 offset = vec2(x, y) / u_screenSize;
            ssao += texture(u_ssao, v_uv + offset * 2.0).r;
        }
    }
    ssao /= 9.0;
    return vec4(texture(u_screen, v_uv).rgb * mix(1.0, ssao, 1.0), 1.0);
}
