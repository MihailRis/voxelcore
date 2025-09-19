#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <string>

#include "typedefs.hpp"

#include "presets/WeatherPreset.hpp"
#include "world/Weather.hpp"
#include "window/Camera.hpp"

class Level;
class Player;
class Camera;
class Batch3D;
class LineBatch;
class ChunksRenderer;
class ParticlesRenderer;
class BlockWrapsRenderer;
class PrecipitationRenderer;
class HandsRenderer;
class NamedSkeletons;
class LinesRenderer;
class TextsRenderer;
class Shader;
class Frustum;
class Engine;
class LevelFrontend;
class Skybox;
class PostProcessing;
class DrawContext;
class ModelBatch;
class Assets;
class Shadows;
class GBuffer;
class DebugLinesRenderer;
struct EngineSettings;

struct CompileTimeShaderSettings {
    bool advancedRender = false;
    bool shadows = false;
    bool ssao = false;
};

class WorldRenderer {
    Engine& engine;
    const Level& level;
    Player& player;
    const Assets& assets;
    std::unique_ptr<Frustum> frustumCulling;
    std::unique_ptr<LineBatch> lineBatch;
    std::unique_ptr<Batch3D> batch3d;
    std::unique_ptr<ModelBatch> modelBatch;
    std::unique_ptr<ChunksRenderer> chunksRenderer;
    std::unique_ptr<HandsRenderer> hands;
    std::unique_ptr<Skybox> skybox;
    std::unique_ptr<Shadows> shadowMapping;
    std::unique_ptr<DebugLinesRenderer> debugLines;
    Weather weather {};
    
    float timer = 0.0f;
    bool debug = false;
    bool lightsDebug = false;
    bool gbufferPipeline = false;

    CompileTimeShaderSettings prevCTShaderSettings {};

    /// @brief Render block selection lines
    void renderBlockSelection();
    
    /// @brief Render lines (selection and debug)
    /// @param camera active camera
    /// @param linesShader shader used
    void renderLines(
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
        float delta,
        bool pause,
        bool hudVisible
    );
public:
    std::unique_ptr<ParticlesRenderer> particles;
    std::unique_ptr<TextsRenderer> texts;
    std::unique_ptr<BlockWrapsRenderer> blockWraps;
    std::unique_ptr<PrecipitationRenderer> precipitation;
    std::unique_ptr<NamedSkeletons> skeletons;
    std::unique_ptr<LinesRenderer> lines;

    static bool showChunkBorders;
    static bool showEntitiesDebug;

    WorldRenderer(Engine& engine, LevelFrontend& frontend, Player& player);
    ~WorldRenderer();

    void renderFrame(
        const DrawContext& context, 
        Camera& camera, 
        bool hudVisible,
        bool pause,
        float delta,
        PostProcessing& postProcessing
    );

    void clear();

    void setDebug(bool flag);

    void toggleLightsDebug();

    Weather& getWeather();
};
