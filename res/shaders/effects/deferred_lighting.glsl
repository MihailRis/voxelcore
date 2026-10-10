#include <shadows>
#include <fog>

#define CLOUDS_FOG_FACTOR_MUL 0.3f
#define CLOUDS_FOG_CURVE_MUL 0.4f
#define COLOR_TEMPERATURE_COMPENSATION 1.14f

vec4 effect() {
    vec4 pos = texture(u_position, v_uv);
    float light = 1.0;

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
    light = sum / wsum;
#endif // ENABLE_SSAO

    vec4 modelpos = u_inverseView * pos;
    vec3 normal = transpose(mat3(u_view)) * texture(u_normal, v_uv).xyz;
    vec3 dir = modelpos.xyz - u_cameraPos;

    float emission = texture(u_emission, v_uv).r;

    float rg = 1.0;
#ifdef ENABLE_SHADOWS
    float shadow_mul = calc_shadow(modelpos, normal, length(pos));
    light *= shadow_mul;
    rg *= min(1.0, shadow_mul + emission) * COLOR_TEMPERATURE_COMPENSATION;
    light *= 2.0;
#endif

    light = max(light, emission);

    light = pow(light, u_gamma);

    vec3 fogColor = texture(u_skybox, dir).rgb;
    float fog = calc_fog(
        length(u_view * vec4((modelpos.xyz - u_cameraPos) * FOG_POS_SCALE, 0.0)) / 256.0,
        mix(1.0, CLOUDS_FOG_FACTOR_MUL, emission),
        mix(1.0, CLOUDS_FOG_CURVE_MUL, emission)
    );
    return vec4(mix(texture(u_screen, v_uv).rgb * mix(1.0, light, 1.0) * vec3(rg, 0.5 + rg * 0.5, 1.0), fogColor, fog), 1.0);
}
