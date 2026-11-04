#include <deferred_light>
#include <fog>

#define CLOUDS_FOG_FACTOR_MUL 0.3f
#define CLOUDS_FOG_CURVE_MUL 0.4f

vec4 effect() {
    vec4 pos = texture(u_position, v_uv);
    float ao = 1.0;

#ifdef ENABLE_SSAO
    vec2 texel = 2.0 / u_screenSize;
    float centerZ = pos.z;
    float sum = 0.0;
    float wsum = 0.0;
    for (int y = -2; y < 2; y++) {
        for (int x = -2; x < 2; x++) {
            vec2 uv = v_uv + vec2(x, y) * texel;
            float z = texture(u_position, uv).z;
            float w = exp(-abs(centerZ - z) / (abs(centerZ) * 0.05 + 0.001));
            sum  += texture(u_ssao, uv).r * w;
            wsum += w;
        }
    }
    ao = sum / wsum;
#endif // ENABLE_SSAO

    vec4 modelpos = u_inverseView * pos;
    vec3 normal = transpose(mat3(u_view)) * texture(u_normal, v_uv).xyz;
    vec3 dir = modelpos.xyz - u_cameraPos;

    float emission = texture(u_emission, v_uv).r;
    vec3 light = calc_deferred_light(modelpos, normal, length(pos), emission, ao);

    vec3 fogColor = texture(u_skybox, dir).rgb;
    float fog = calc_fog(
        length(u_view * vec4((modelpos.xyz - u_cameraPos) * FOG_POS_SCALE, 0.0)) / 256.0,
        mix(1.0, CLOUDS_FOG_FACTOR_MUL, emission),
        mix(1.0, CLOUDS_FOG_CURVE_MUL, emission)
    );
    return vec4(mix(texture(u_screen, v_uv).rgb * light, fogColor, fog), 1.0);
}
