#pragma once

#include "commons.hpp"

#include <array>
#include <memory>
#include <vector>

class Camera;
class Shader;
struct Weather;
class Frustum;

class CloudsRenderer final {
public:
    CloudsRenderer();
    ~CloudsRenderer();

    void draw(
        Shader& shader,
        const Weather& weather,
        float timer,
        float fogFactor,
        const Camera& camera,
        int quality
    );
private:
    struct Layer {
        int diameter;
        int segmentSize;
        std::vector<std::unique_ptr<Mesh<ChunkVertex>>> meshes;
    };

    std::array<Layer, 2> layers;

    void draw(
        Layer& layer,
        Frustum& frustum,
        Shader& shader,
        const Camera& camera,
        float timer,
        int layerId
    );
};
