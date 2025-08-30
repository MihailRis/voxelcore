vec4 effect() {
    vec3 color = texture(u_screen, v_uv).rgb;
    float m = (color.r + color.g + color.b) / 3.0;
    return vec4(mix(color, vec3(m), u_intensity), 1.0);
}
