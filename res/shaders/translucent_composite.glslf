in vec2 v_uv;
out vec4 f_color;

#define LAYERS_COUNT 3

uniform sampler2D u_layerColor0;
uniform sampler2D u_layerColor1;
uniform sampler2D u_layerColor2;
uniform sampler2D u_layerDepth0;
uniform sampler2D u_layerDepth1;
uniform sampler2D u_layerDepth2;

void main() {
    vec4 colors[LAYERS_COUNT];
    float depths[LAYERS_COUNT];
    colors[0] = texture(u_layerColor0, v_uv);
    colors[1] = texture(u_layerColor1, v_uv);
    colors[2] = texture(u_layerColor2, v_uv);
    depths[0] = texture(u_layerDepth0, v_uv).r;
    depths[1] = texture(u_layerDepth1, v_uv).r;
    depths[2] = texture(u_layerDepth2, v_uv).r;

    vec4 layerColors[LAYERS_COUNT];
    float layerDepths[LAYERS_COUNT];
    int count = 0;
    for (int i = 0; i < LAYERS_COUNT; i++) {
        if (colors[i].a > 0.0) {
            layerColors[count] = colors[i];
            layerDepths[count] = depths[i];
            count++;
        }
    }
    if (count == 0) {
        discard;
    }

    for (int i = 1; i < count; i++) {
        vec4 color = layerColors[i];
        float depth = layerDepths[i];
        int j = i - 1;
        while (j >= 0 && layerDepths[j] < depth) {
            layerColors[j + 1] = layerColors[j];
            layerDepths[j + 1] = layerDepths[j];
            j--;
        }
        layerColors[j + 1] = color;
        layerDepths[j + 1] = depth;
    }

    vec4 result = vec4(0.0);
    for (int i = 0; i < count; i++) {
        result = result * (1.0 - layerColors[i].a) + layerColors[i];
    }
    f_color = result;
}
