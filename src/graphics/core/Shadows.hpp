#pragma once

#include <memory>
#include <functional>

#include "typedefs.hpp"
#include "window/Camera.hpp"

class Shader;
class Level;
class Assets;
struct Weather;
class DrawContext;
struct EngineSettings;
class ShadowMap;

class Shadows {
public:
    Shadows(const Level& level);
    ~Shadows();

    void setup(Shader& shader, const Weather& weather);
    void setQuality(int quality);
    void refresh(
        const Camera& camera,
        const DrawContext& pctx,
        std::function<void(Camera&)> renderShadowPass
    );
private:
    const Level& level;
    bool shadows = false;
    Camera shadowCamera;
    Camera wideShadowCamera;
    std::unique_ptr<ShadowMap> shadowMap;
    std::unique_ptr<ShadowMap> wideShadowMap;
    int quality = 0;

    void generateShadowsMap(
        const Camera& camera,
        const DrawContext& pctx,
        ShadowMap& shadowMap,
        Camera& shadowCamera,
        float scale,
        std::function<void(Camera&)> renderShadowPass
    );
};
