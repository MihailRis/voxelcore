#pragma once

#include "commons.hpp"
#include "typedefs.hpp"

#include "presets/WeatherPreset.hpp"
#include "window/Camera.hpp"
#include "util/ObjectsKeeper.hpp"

#include <vector>
#include <memory>
#include <string>

class Assets;
class Batch3D;
class BlockWrapsRenderer;
class ChunksRenderer;
class DebugLinesRenderer;
class DrawContext;
class Engine;
class Frustum;
class HandsRenderer;
class Level;
class LevelFrontend;
class LineBatch;
class ModelBatch;
class NamedSkeletons;
class ParticlesRenderer;
class Player;
class PostProcessing;
class PrecipitationRenderer;
class Shader;
class Shadows;
class Skybox;
class TextsRenderer;
class CloudsRenderer;
struct EngineSettings;
struct Weather;

class WorldRenderer final : public util::ObjectsKeeper {
public:
    static bool showChunkBorders;
    static bool showEntitiesDebug;

    WorldRenderer(Engine& engine, LevelFrontend& frontend, Player& player);
    ~WorldRenderer();

    void update(const Camera& camera, float delta);

    void renderFrame(
        const DrawContext& context, 
        Camera& camera, 
        bool hudVisible,
        PostProcessing& postProcessing
    );

    void resetCache();

    void setDebug(bool flag);

    void toggleLightsDebug();

    Weather& getWeather();
private:
    Engine& engine;
    const Level& level;
    Player& player;
    const Assets& assets;
    Weather& weather;
    std::unique_ptr<Frustum> frustumCulling;
    std::unique_ptr<LineBatch> lineBatch;
    std::unique_ptr<Batch3D> batch3d;
    std::unique_ptr<ModelBatch> modelBatch;
    std::unique_ptr<ChunksRenderer> chunksRenderer;
    std::unique_ptr<HandsRenderer> hands;
    std::unique_ptr<Skybox> skybox;
    std::unique_ptr<Shadows> shadowMapping;
    std::unique_ptr<DebugLinesRenderer> debugLines;
    std::unique_ptr<PrecipitationRenderer> precipitation;
    std::unique_ptr<CloudsRenderer> cloudsRenderer;
    
    float timer = 0.0f;
    bool debug = false;
    bool lightsDebug = false;
    bool gbufferPipeline = false;
    bool dirtySettings = true;

    /// @brief Render block selection lines
    void renderBlockSelection();
    
    /// @brief Render lines (selection and debug)
    /// @param camera active camera
    /// @param linesShader shader used
    void renderInWorldLines(
        const Camera& camera, Shader& linesShader, const DrawContext& pctx
    );

    void renderBlockOverlay(const DrawContext& context);

    void setupWorldShader(
        Shader& shader,
        const Camera& camera,
        const EngineSettings& settings,
        float fogFactor
    );

    /// @brief Render opaque pass
    /// @param context graphics context
    /// @param camera active camera
    /// @param settings engine settings
    void renderOpaque(
        const DrawContext& context, 
        const Camera& camera, 
        const EngineSettings& settings,
        bool hudVisible
    );

    void renderOpaquePass(
        const DrawContext& context,
        Camera& camera,
        bool hudVisible,
        PostProcessing& postProcessing
    );

    void renderWeatherEffects(Camera& camera);

    void renderHandsPass(const DrawContext& pctx, Camera& camera);

    void renderDebugLines(const DrawContext& context, Camera& camera);

    void renderFrameClassic(
        const DrawContext& context, 
        Camera& camera, 
        bool hudVisible,
        PostProcessing& postProcessing
    );

    void renderFrameAdvanced(
        const DrawContext& context, 
        Camera& camera, 
        bool hudVisible,
        PostProcessing& postProcessing
    );

    void refreshSettings();

    float calcFogFactor() const;
public:
    std::unique_ptr<ParticlesRenderer> particles;
    std::unique_ptr<TextsRenderer> texts;
    std::unique_ptr<BlockWrapsRenderer> blockWraps;
    std::unique_ptr<NamedSkeletons> skeletons;
};
