vec4 effect() {
    vec4 color = texture(u_screen, v_uv);
    color = mix(color, 1.0 - color, u_intensity);
    color.a = 1.0;
    return color;
}
