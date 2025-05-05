uniform vec3 samples[64];

int kernelSize = 32;
float radius = 0.25;
float bias = 0.025;

float near_plane = 0.5f;
float far_plane = 200.0f;

// required when using a perspective projection matrix
float linearize_depth(float depth) {
    float z = depth * 2.0 - 1.0; // Back to NDC 
    return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));	
}

float fmod(float x, float y) {
    return x - y * floor(x/y);
}

vec4 effect() {
    vec2 noiseScale = u_screenSize / 4.0;

    vec3 position = texture(u_position, v_uv).xyz;
    vec3 color = texture(u_screen, v_uv).rgb;
    vec3 normal = texture(u_normal, v_uv).xyz;
    vec3 randomVec = normalize(texture(u_noise, v_uv * noiseScale).xyz);

    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN  = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    for(int i = 0; i < kernelSize; ++i)
    {
        vec3 samplePos = TBN * samples[i];
        samplePos = position + samplePos * radius; 
        
        vec4 offset = vec4(samplePos, 1.0);
        offset = u_projection * offset;
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5 + 0.5;
        
        float sampleDepth = texture(u_position, offset.xy).z;
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(position.z - sampleDepth));
        occlusion += (sampleDepth >= samplePos.z + bias ? 1.0 : 0.0) * rangeCheck;    
    }
    occlusion = 1.1 - (occlusion / kernelSize);
    return vec4(color * occlusion, 1.0);
}
