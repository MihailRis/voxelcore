#param vec3 u_ssaoSamples[64]
#param int u_kernelSize = 16
#param float u_radius = 0.4
#param float u_bias = 0.006

vec4 effect() {
    vec2 noiseScale = u_screenSize / 4.0;

    vec3 position = texture(u_position, v_uv).xyz;
    vec3 normal = texture(u_normal, v_uv).xyz;
    vec3 randomVec = normalize(texture(u_noise, v_uv * noiseScale).xyz);

    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 tbn = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    for (int i = 0; i < u_kernelSize; i++) {
        vec3 samplePos = tbn * u_ssaoSamples[i];
        samplePos = position + samplePos * u_radius; 
        
        vec4 offset = vec4(samplePos, 1.0);
        offset = u_projection * offset;
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5 + 0.5;
        
        float sampleDepth = texture(u_position, offset.xy).z;
        float rangeCheck = smoothstep(0.0, 1.0, u_radius / abs(position.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + u_bias ? 1.0 : 0.0) * rangeCheck;
    }
    occlusion = min(1.0, 1.05 - (occlusion / u_kernelSize));
    occlusion = max(occlusion, texture(u_emission, v_uv).r);

    float z = -position.z * 0.01;
    z = max(0.0, 1.0 - z);
    return vec4(mix(1.0, occlusion, z), 0.0, 0.0, 1.0);
}
