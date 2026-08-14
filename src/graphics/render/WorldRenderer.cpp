#include "WorldRenderer.hpp"

#include "animation/rigging.hpp"
#include "BlockWrapsRenderer.hpp"
#include "ChunksRenderer.hpp"
#include "CloudsRenderer.hpp"
#include "DebugLinesRenderer.hpp"
#include "Emitter.hpp"
#include "HandsRenderer.hpp"
#include "ModelBatch.hpp"
#include "NamedSkeletons.hpp"
#include "ParticlesRenderer.hpp"
#include "PrecipitationRenderer.hpp"
#include "Skybox.hpp"
#include "TextNote.hpp"
#include "TextsRenderer.hpp"
#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "coders/GLSLExtension.hpp"
#include "content/Content.hpp"
#include "engine/Engine.hpp"
#include "frontend/ContentGfxCache.hpp"
#include "frontend/LevelFrontend.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/Framebuffer.hpp"
#include "graphics/core/GBuffer.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/PostProcessing.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Shadows.hpp"
#include "graphics/core/Cubemap.hpp"
#include "graphics/core/Texture.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "items/ItemStack.hpp"
#include "logic/PlayerController.hpp"
#include "maths/FrustumCulling.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "settings.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Pathfinding.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"
#include "world/LevelEvents.hpp"
#include "world/Weather.hpp"
#include "world/World.hpp"
#include "debug/Logger.hpp"

#include <assert.h>
#include <algorithm>
#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

inline constexpr size_t BATCH3D_CAPACITY = 4096;
inline constexpr size_t MODEL_BATCH_CAPACITY = 20'000;

static debug::Logger logger("world-renderer");

bool WorldRenderer::showChunkBorders = false;
bool WorldRenderer::showEntitiesDebug = false;

template <typename T>
static ObserverHandler observe_setting(
    ObservableSetting<T>& setting, bool& dirtySettings
) {
    return setting.observe([&dirtySettings](auto const&) {
        dirtySettings = true;
    });
}

WorldRenderer::WorldRenderer(
    Engine& engine, LevelFrontend& frontend, Player& player
)
    : engine(engine),
      level(frontend.getLevel()),
      player(player),
      assets(*engine.getAssets()),
      weather(frontend.getWeather()),
      frustumCulling(std::make_unique<Frustum>()),
      lineBatch(std::make_unique<LineBatch>()),
      batch3d(std::make_unique<Batch3D>(BATCH3D_CAPACITY)),
      modelBatch(std::make_unique<ModelBatch>(
          MODEL_BATCH_CAPACITY, assets, *player.chunks, engine.getSettings()
      )),
      chunksRenderer(std::make_unique<ChunksRenderer>(
          level,
          *player.chunks,
          assets,
          *frustumCulling,
          frontend.getContentGfxCache(),
          engine.getSettings()
      )),
      precipitation(std::make_unique<PrecipitationRenderer>(
          assets, level, *player.chunks, &engine.getSettings().graphics
      )),
      particles(std::make_unique<ParticlesRenderer>(
          assets, level, *player.chunks, engine.getSettings().graphics
      )),
      texts(std::make_unique<TextsRenderer>(
          level, *batch3d, assets, *frustumCulling
      )),
      blockWraps(std::make_unique<BlockWrapsRenderer>(
          assets, level.content, *player.chunks
      )) {
    auto& settings = engine.getSettings();
    level.events->listen(
        LevelEventType::CHUNK_HIDDEN,
        [this](LevelEventType, Chunk* chunk) { chunksRenderer->unload(chunk); }
    );
    skybox = std::make_unique<Skybox>(
        settings.graphics.skyboxResolution.get(), assets
    );

    const auto& content = level.content;
    skeletons = std::make_unique<NamedSkeletons>();
    const auto& skeletonConfig = assets.require<rigging::SkeletonConfig>(
        content.getDefaults()["hand-skeleton"].asString()
    );
    hands = std::make_unique<HandsRenderer>(
        assets,
        *modelBatch,
        skeletons->createSkeleton("hand", skeletonConfig.shared_from_this())
    );
    shadowMapping = std::make_unique<Shadows>(level);
    debugLines = std::make_unique<DebugLinesRenderer>(level);
    cloudsRenderer = std::make_unique<CloudsRenderer>();

    keepAlive(observe_setting(settings.graphics.advancedRender, dirtySettings));
    keepAlive(observe_setting(settings.graphics.shadowsQuality, dirtySettings));
    keepAlive(observe_setting(settings.graphics.ssao, dirtySettings));
}

WorldRenderer::~WorldRenderer() = default;

void WorldRenderer::refreshSettings() {
    Shader* affectedShaders[] {
        &assets.require<Shader>("main"),
        &assets.require<Shader>("entity"),
        &assets.require<Shader>("clouds"),
        &assets.require<Shader>("translucent"),
        &assets.require<PostEffect>("deferred_lighting").getShader(),
        nullptr
    };

    const auto& graphics = engine.getSettings().graphics;
    gbufferPipeline = graphics.advancedRender.get();

    int shadowsQuality = graphics.shadowsQuality.get() * gbufferPipeline;
    shadowMapping->setQuality(shadowsQuality);

    std::vector<std::string> defines;
    if (shadowsQuality != 0) defines.emplace_back("ENABLE_SHADOWS");
    if (graphics.ssao.get()) defines.emplace_back("ENABLE_SSAO");
    if (gbufferPipeline) defines.emplace_back("ADVANCED_RENDER");

    logger.info() << "recompiling shaders due to settings change";
    for (size_t i = 0; affectedShaders[i]; i++) {
        affectedShaders[i]->recompile(defines);
    }
}

static void setup_weather(Shader& shader, const Weather& weather) {
    shader.uniform1f("u_weatherFogOpacity", weather.fogOpacity());
    shader.uniform1f("u_weatherFogDencity", weather.fogDencity());
    shader.uniform1f("u_weatherFogCurve", weather.fogCurve());
    shader.uniform3f("u_minSkyLight", weather.minSkyLight());
}

static void setup_camera(Shader& shader, const Camera& camera) {
    shader.uniformMatrix("u_model", glm::mat4(1.0f));
    shader.uniformMatrix("u_proj", camera.getProjection());
    shader.uniformMatrix("u_view", camera.getView());
    shader.uniform3f("u_cameraPos", camera.position);
}

void WorldRenderer::setupWorldShader(
    Shader& shader,
    const Camera& camera,
    const EngineSettings& settings,
    float fogFactor
) {
    shader.use();

    setup_camera(shader, camera);
    setup_weather(shader, weather);
    shadowMapping->setup(shader, weather);

    shader.uniform1f("u_timer", timer);
    shader.uniform1f("u_gamma", settings.graphics.gamma.get());
    shader.uniform1f("u_fogFactor", fogFactor);
    shader.uniform1f("u_fogCurve", settings.graphics.fogCurve.get());
    shader.uniform1i("u_debugLights", lightsDebug);
    shader.uniform1i("u_debugNormals", false);
    shader.uniform1f("u_dayTime", level.getWorld().getInfo().daytime);
    shader.uniform2f("u_lightDir", skybox->getLightDir());
    shader.uniform1i("u_skybox", advanced_pipeline::TARGET_SKYBOX);

    auto indices = level.content.getIndices();
    // Light emission when an emissive item is chosen
    {
        auto inventory = player.getInventory();
        ItemStack& stack = inventory->getSlot(player.getChosenSlot());
        auto& item = indices->items.require(stack.getItemId());
        float multiplier = 0.75f;
        shader.uniform3f(
            "u_torchlightColor",
            item.emission[0] / 15.0f * multiplier,
            item.emission[1] / 15.0f * multiplier,
            item.emission[2] / 15.0f * multiplier
        );
        shader.uniform1f("u_torchlightDistance", 8.0f);
    }
}

float WorldRenderer::calcFogFactor() const {
    const auto& settings = engine.getSettings();
    return 15.0f / static_cast<float>(settings.chunks.loadDistance.get() - 2);
}

void WorldRenderer::renderOpaque(
    const DrawContext& ctx,
    const Camera& camera,
    const EngineSettings& settings,
    bool hudVisible
) {
    float fogFactor = calcFogFactor();

    auto& entityShader = assets.require<Shader>("entity");
    setupWorldShader(entityShader, camera, settings, fogFactor);

    bool culling = engine.getSettings().graphics.frustumCulling.get();
    if (culling) {
        frustumCulling->update(camera.getProjView());
    }

    entityShader.uniform1i("u_alphaClip", true);
    entityShader.uniform1f("u_opacity", 1.0f);
    level.entities->render(
        assets,
        *modelBatch,
        culling ? frustumCulling.get() : nullptr,
        player.currentCamera.get() == player.fpCamera.get() ? player.getEntity()
                                                            : 0
    );
    modelBatch->render();
    particles->render(camera);

    auto& mainShader = assets.require<Shader>("main");
    auto& cloudsShader = assets.require<Shader>("clouds");

    setupWorldShader(mainShader, camera, settings, fogFactor);

    chunksRenderer->drawChunks(camera, mainShader);
    blockWraps->draw(ctx);

    if (level.environment.sky.clouds) {
        int cloudsQuality = settings.graphics.cloudsQuality.get();
        if (cloudsQuality > 0) {
            setupWorldShader(cloudsShader, camera, settings, fogFactor);
            cloudsRenderer->draw(
                cloudsShader, weather, timer, fogFactor, camera, cloudsQuality
            );
        }
    }

    if (hudVisible) {
        auto& linesShader = assets.require<Shader>("lines");
        renderInWorldLines(camera, linesShader, ctx);
    }
}

void WorldRenderer::renderBlockSelection() {
    const auto& selection = player.selection;
    auto indices = level.content.getIndices();
    blockid_t id = selection.vox.id;
    auto& block = indices->blocks.require(id);
    const glm::ivec3 pos = player.selection.position;
    const glm::vec3 point = selection.hitPosition;
    const glm::vec3 norm = selection.normal;

    const std::vector<AABB>& hitboxes =
        block.rotatable ? block.rt.hitboxes[selection.vox.state.rotation]
                        : block.hitboxes;

    lineBatch->lineWidth(2.0f);
    for (auto& hitbox : hitboxes) {
        const glm::vec3 center = glm::vec3(pos) + hitbox.center();
        const glm::vec3 size = hitbox.size();
        lineBatch->box(
            center, size + glm::vec3(0.01), glm::vec4(0.f, 0.f, 0.f, 1.0f)
        );
        if (debug) {
            lineBatch->line(
                point, point + norm * 0.5f, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f)
            );
        }
    }
    lineBatch->flush();
}

void WorldRenderer::renderInWorldLines(
    const Camera& camera, Shader& linesShader, const DrawContext& pctx
) {
    linesShader.use();
    linesShader.uniformMatrix("u_projview", camera.getProjView());
    if (player.selection.vox.id != BLOCK_VOID) {
        renderBlockSelection();
    }
    if (debug && showEntitiesDebug) {
        auto ctx = pctx.sub(lineBatch.get());
        bool culling = engine.getSettings().graphics.frustumCulling.get();
        level.entities->renderDebug(
            *lineBatch, culling ? frustumCulling.get() : nullptr, ctx
        );
    }
}

void WorldRenderer::update(const Camera& camera, float delta) {
    timer += delta;
    chunksRenderer->update();
    weather.update(delta);
    precipitation->update(delta);
    particles->update(camera, delta);
}

void WorldRenderer::renderFrameClassic(
    const DrawContext& pctx,
    Camera& camera,
    bool hudVisible,
    PostProcessing& postProcessing
) {
    const auto& settings = engine.getSettings();
    const auto& worldInfo = level.getWorld().getInfo();

    DrawContext ctx = pctx.sub();
    ctx.setDepthTest(true);
    ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
    ctx.setFramebuffer(postProcessing.getFramebuffer());

    skybox->draw(
        level.environment, ctx, camera, worldInfo.daytime, weather.clouds()
    );
    texts->render(ctx, camera, settings, hudVisible, false);

    if (debug && hudVisible) {
        renderDebugLines(ctx, camera);
    }

    DrawContext wctx = ctx.sub();
    wctx.useTexture(advanced_pipeline::TARGET_SKYBOX, skybox->getCubemap());
    wctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
    // Translucent blocks
    {
        const auto& settings = engine.getSettings();
        auto& translucentShader = assets.require<Shader>("translucent");
        auto sctx = wctx.sub();
        sctx.setCullFace(true);
        translucentShader.use();
        setupWorldShader(translucentShader, camera, settings, calcFogFactor());
        chunksRenderer->drawSortedMeshes(camera, translucentShader);
    }

    renderWeatherEffects(camera);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void WorldRenderer::renderFrameAdvanced(
    const DrawContext& pctx,
    Camera& camera,
    bool hudVisible,
    PostProcessing& postProcessing
) {
    const auto& settings = engine.getSettings();
    const auto& worldInfo = level.getWorld().getInfo();
    float fogFactor = calcFogFactor();

    shadowMapping->refresh(camera, pctx, [this, &camera](Camera& shadowCamera) {
        auto& shader = assets.require<Shader>("shadows");
        setupWorldShader(shader, shadowCamera, engine.getSettings(), 0.0f);
        chunksRenderer->drawShadowsPass(shadowCamera, shader, camera);
    });

    auto& deferredShader =
        assets.require<PostEffect>("deferred_lighting").getShader();
    deferredShader.use();
    setupWorldShader(deferredShader, camera, settings, fogFactor);
    postProcessing.renderDeferredShading(pctx, assets, timer, camera);

    DrawContext ctx = pctx.sub();
    ctx.setDepthTest(true);
    ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);

    postProcessing.bindDepthBuffer();
    skybox->draw(
        level.environment, ctx, camera, worldInfo.daytime, weather.clouds()
    );
    texts->render(ctx, camera, settings, hudVisible, false);
    if (debug && hudVisible) {
        renderDebugLines(ctx, camera);
    }

    DrawContext wctx = ctx.sub();
    wctx.useTexture(advanced_pipeline::TARGET_SKYBOX, skybox->getCubemap());
    wctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
    // Translucent blocks
    {
        const auto& settings = engine.getSettings();
        auto& translucentShader = assets.require<Shader>("translucent");
        auto sctx = wctx.sub();
        sctx.setCullFace(true);
        translucentShader.use();
        setupWorldShader(translucentShader, camera, settings, fogFactor);
        chunksRenderer->drawSortedMeshes(camera, translucentShader);
    }

    renderWeatherEffects(camera);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void WorldRenderer::renderFrame(
    const DrawContext& pctx,
    Camera& camera,
    bool hudVisible,
    PostProcessing& postProcessing
) {
    const auto& vp = pctx.getViewport();
    camera.setAspectRatio(vp.x / static_cast<float>(vp.y));
    if (dirtySettings) {
        refreshSettings();
        dirtySettings = false;
    }

    const auto& worldInfo = level.getWorld().getInfo();
    float clouds = weather.clouds();

    float random = rand() / static_cast<float>(RAND_MAX);
    skybox->refresh(
        level.environment,
        pctx,
        worldInfo.daytime,
        1.0f + clouds,
        weather.skyTint(),
        weather.highlight * random,
        4
    );
    renderOpaquePass(pctx, camera, hudVisible, postProcessing);
    if (gbufferPipeline) {
        renderFrameAdvanced(pctx, camera, hudVisible, postProcessing);
    } else {
        renderFrameClassic(pctx, camera, hudVisible, postProcessing);
    }
    postProcessing.render(pctx, assets, timer, camera);

    if (player.currentCamera == player.fpCamera) {
        renderHandsPass(pctx, camera);
    }
    {
        auto bctx = pctx.sub();
        bctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
        renderBlockOverlay(pctx);
    }
    glActiveTexture(GL_TEXTURE0);
}


void WorldRenderer::renderOpaquePass(
    const DrawContext& pctx,
    Camera& camera, 
    bool hudVisible,
    PostProcessing& postProcessing
) {
    const auto& settings = engine.getSettings();

    DrawContext wctx = pctx.sub();
    postProcessing.use(wctx, gbufferPipeline);
    display::clearDepth();

    /* Main opaque pass (GBuffer pass) */ {
        DrawContext ctx = wctx.sub();
        ctx.setDepthTest(true);
        ctx.setCullFace(true);
        ctx.useTexture(advanced_pipeline::TARGET_SKYBOX, skybox->getCubemap());
        ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
        renderOpaque(ctx, camera, settings, hudVisible);

        ctx.setDepthMask(false);
        texts->render(wctx, camera, settings, hudVisible, true);
    }
}

void WorldRenderer::renderWeatherEffects(Camera& camera) {
    const auto& settings = engine.getSettings();
    auto& entityShader = assets.require<Shader>("entity");
    entityShader.use();
    setupWorldShader(entityShader, camera, settings, calcFogFactor());

    std::array<const WeatherPreset*, 2> weatherInstances {
        &weather.a, &weather.b};
    for (const auto& weather : weatherInstances) {
        float maxIntensity = weather->fall.maxIntensity;
        float zero = weather->fall.minOpacity;
        float one = weather->fall.maxOpacity;
        float t = (weather->intensity * (one - zero)) * maxIntensity + zero;
        entityShader.uniform1i("u_alphaClip", weather->fall.opaque);
        entityShader.uniform1f(
            "u_opacity", weather->fall.opaque ? t * t : t
        );
        if (weather->intensity > 1.e-3f && !weather->fall.texture.empty()) {
            precipitation->render(camera, *weather);
        }
    }
}

void WorldRenderer::renderDebugLines(const DrawContext& ctx, Camera& camera) {
    auto& linesShader = assets.require<Shader>("lines");
    linesShader.use();
    debugLines->render(
        ctx, camera, *lineBatch, linesShader, showChunkBorders
    );
    linesShader.uniformMatrix("u_projview", camera.getProjView());
    lineBatch->flush();
}

void WorldRenderer::renderHandsPass(const DrawContext& pctx, Camera& camera) {
    auto& entityShader = assets.require<Shader>("entity");

    DrawContext ctx = pctx.sub();
    ctx.setDepthTest(true);
    ctx.setCullFace(true);
    ctx.useTexture(advanced_pipeline::TARGET_SKYBOX, skybox->getCubemap());
    ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);
    display::clearDepth();

    // prepare modified HUD camera
    Camera hudcam = camera;
    hudcam.far = 10.0f;
    hudcam.setFov(0.9f);
    hudcam.position = {};

    setupWorldShader(entityShader, hudcam, engine.getSettings(), 0.0f);
    hands->render(camera);
}

void WorldRenderer::renderBlockOverlay(const DrawContext& wctx) {
    int x = std::floor(player.currentCamera->position.x);
    int y = std::floor(player.currentCamera->position.y);
    int z = std::floor(player.currentCamera->position.z);
    auto block = player.chunks->get(x, y, z);

    if (block == nullptr || block->id == BLOCK_AIR || block->id == BLOCK_VOID) {
        return;
    }
    const auto& def = level.content.getIndices()->blocks.require(block->id);
    if (def.overlayTexture.empty()) {
        return;
    }
    auto textureRegion =
        util::get_texture_region(assets, def.overlayTexture, "blocks:notfound");
    DrawContext ctx = wctx.sub();
    ctx.setDepthTest(false);
    ctx.setCullFace(false);

    auto& shader = assets.require<Shader>("ui3d");
    shader.use();
    batch3d->begin();
    shader.uniformMatrix("u_projview", glm::mat4(1.0f));
    shader.uniformMatrix("u_apply", glm::mat4(1.0f));
    auto light = player.chunks->getLight(x, y, z);
    float s = Lightmap::extract(light, 3) / 15.0f;
    glm::vec4 tint(
        glm::min(1.0f, Lightmap::extract(light, 0) / 15.0f + s),
        glm::min(1.0f, Lightmap::extract(light, 1) / 15.0f + s),
        glm::min(1.0f, Lightmap::extract(light, 2) / 15.0f + s),
        1.0f
    );
    batch3d->texture(textureRegion.texture);
    batch3d->sprite(
        glm::vec3(),
        glm::vec3(0, 1, 0),
        glm::vec3(1, 0, 0),
        2,
        2,
        textureRegion.region,
        tint
    );
    batch3d->flush();
}

void WorldRenderer::resetCache() {
    chunksRenderer->clear();
}

void WorldRenderer::setDebug(bool flag) {
    debug = flag;
}

void WorldRenderer::toggleLightsDebug() {
    lightsDebug = !lightsDebug;
}

Weather& WorldRenderer::getWeather() {
    return weather;
}
