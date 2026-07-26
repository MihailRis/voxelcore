#include "WorldRenderer.hpp"

#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "content/Content.hpp"
#include "engine/Engine.hpp"
#include "coders/GLSLExtension.hpp"
#include "frontend/LevelFrontend.hpp"
#include "frontend/ContentGfxCache.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "items/ItemStack.hpp"
#include "logic/PlayerController.hpp"
#include "maths/FrustumCulling.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "objects/rigging.hpp"
#include "settings.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Pathfinding.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"
#include "world/LevelEvents.hpp"
#include "world/World.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/PostProcessing.hpp"
#include "graphics/core/Framebuffer.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/Shadows.hpp"
#include "graphics/core/GBuffer.hpp"
#include "BlockWrapsRenderer.hpp"
#include "ParticlesRenderer.hpp"
#include "PrecipitationRenderer.hpp"
#include "HandsRenderer.hpp"
#include "NamedSkeletons.hpp"
#include "TextsRenderer.hpp"
#include "ChunksRenderer.hpp"
#include "DebugLinesRenderer.hpp"
#include "ModelBatch.hpp"
#include "Skybox.hpp"
#include "Emitter.hpp"
#include "TextNote.hpp"
#include "CloudsRenderer.hpp"

#include <assert.h>
#include <algorithm>
#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

using namespace advanced_pipeline;

inline constexpr size_t BATCH3D_CAPACITY = 4096;
inline constexpr size_t MODEL_BATCH_CAPACITY = 20'000;

bool WorldRenderer::showChunkBorders = false;
bool WorldRenderer::showEntitiesDebug = false;

WorldRenderer::WorldRenderer(
    Engine& engine, LevelFrontend& frontend, Player& player
)
    : engine(engine),
      level(frontend.getLevel()),
      player(player),
      assets(*engine.getAssets()),
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
        settings.graphics.skyboxResolution.get(),
        assets.require<Shader>("skybox_gen")
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
}

WorldRenderer::~WorldRenderer() = default;

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
    shader.uniform1f("u_dayTime", level.getWorld()->getInfo().daytime);
    shader.uniform2f("u_lightDir", skybox->getLightDir());
    shader.uniform1i("u_skybox", TARGET_SKYBOX);

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

void WorldRenderer::renderOpaque(
    const DrawContext& ctx,
    const Camera& camera,
    const EngineSettings& settings,
    bool hudVisible
) {
    texts->render(ctx, camera, settings, hudVisible, false);

    bool culling = engine.getSettings().graphics.frustumCulling.get();
    float fogFactor =
        15.0f / static_cast<float>(settings.chunks.loadDistance.get() - 2);

    auto& entityShader = assets.require<Shader>("entity");
    setupWorldShader(entityShader, camera, settings, fogFactor);
    skybox->bind();

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

    auto& shader = assets.require<Shader>("main");
    auto& cloudsShader = assets.require<Shader>("clouds");
    auto& linesShader = assets.require<Shader>("lines");

    setupWorldShader(shader, camera, settings, fogFactor);

    chunksRenderer->drawChunks(camera, shader);
    blockWraps->draw(ctx);

    int cloudsQuality = settings.graphics.cloudsQuality.get();
    if (cloudsQuality > 0) {
        setupWorldShader(cloudsShader, camera, settings, fogFactor);
        cloudsRenderer->draw(
            cloudsShader, weather, timer, fogFactor, camera, cloudsQuality
        );
    }

    if (hudVisible) {
        renderLines(camera, linesShader, ctx);
    }
    skybox->unbind();
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

void WorldRenderer::renderLines(
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

void WorldRenderer::refreshSettings(Shader** shaders) {
    const auto& graphics = engine.getSettings().graphics;
    gbufferPipeline = graphics.advancedRender.get();

    int shadowsQuality = graphics.shadowsQuality.get() * gbufferPipeline;
    shadowMapping->setQuality(shadowsQuality);
    
    CompileTimeShaderSettings currentSettings {
        gbufferPipeline,
        shadowsQuality != 0,
        graphics.ssao.get() && gbufferPipeline
    };
    if (
        prevCTShaderSettings.advancedRender != currentSettings.advancedRender ||
        prevCTShaderSettings.shadows != currentSettings.shadows ||
        prevCTShaderSettings.ssao != currentSettings.ssao
    ) {
        std::vector<std::string> defines;
        if (currentSettings.shadows) defines.emplace_back("ENABLE_SHADOWS");
        if (currentSettings.ssao) defines.emplace_back("ENABLE_SSAO");
        if (currentSettings.advancedRender) defines.emplace_back("ADVANCED_RENDER");

        for (size_t i = 0; shaders[i]; i++) {
            shaders[i]->recompile(defines);
        }
        prevCTShaderSettings = currentSettings;
    }
}

void WorldRenderer::update(const Camera& camera, float delta) {
    timer += delta;
    weather.update(delta);
    precipitation->update(delta);
    particles->update(camera, delta);
}

void WorldRenderer::renderFrame(
    const DrawContext& pctx,
    Camera& camera,
    bool hudVisible,
    PostProcessing& postProcessing
) {
    auto projView = camera.getProjView();
    auto world = level.getWorld();

    const auto& vp = pctx.getViewport();
    camera.setAspectRatio(vp.x / static_cast<float>(vp.y));

    auto& mainShader = assets.require<Shader>("main");
    auto& entityShader = assets.require<Shader>("entity");
    auto& cloudsShader = assets.require<Shader>("clouds");
    auto& translucentShader = assets.require<Shader>("translucent");
    auto& deferredShader = assets.require<PostEffect>("deferred_lighting").getShader();

    const auto& settings = engine.getSettings();

    Shader* affectedShaders[] {
        &mainShader,
        &entityShader,
        &cloudsShader,
        &translucentShader,
        &deferredShader,
        nullptr
    };

    refreshSettings(affectedShaders);

    const auto& worldInfo = world->getInfo();
    
    float clouds = weather.clouds();
    clouds = glm::max(worldInfo.fog, clouds);
    float mie = 1.0f + glm::max(worldInfo.fog, clouds * 0.5f) * 2.0f;

    float random = rand() / static_cast<float>(RAND_MAX);
    skybox->refresh(
        pctx,
        worldInfo.daytime,
        mie,
        weather.skyTint(),
        weather.highlight * random,
        4
    );

    chunksRenderer->update();

    shadowMapping->refresh(camera, pctx, [this, &camera](Camera& shadowCamera) {
        auto& shader = assets.require<Shader>("shadows");
        setupWorldShader(shader, shadowCamera, engine.getSettings(), 0.0f);
        chunksRenderer->drawShadowsPass(shadowCamera, shader, camera);
    });
    {
        DrawContext wctx = pctx.sub();
        postProcessing.use(wctx, gbufferPipeline);

        display::clearDepth();

        /* Main opaque pass (GBuffer pass) */ {
            DrawContext ctx = wctx.sub();
            ctx.setDepthTest(true);
            ctx.setCullFace(true);
            renderOpaque(ctx, camera, settings, hudVisible);
        }
        texts->render(pctx, camera, settings, hudVisible, true);
    }
    skybox->bind();
    float fogFactor =
        15.0f / static_cast<float>(settings.chunks.loadDistance.get() - 2);
    if (gbufferPipeline) {
        deferredShader.use();
        setupWorldShader(deferredShader, camera, settings, fogFactor);
        postProcessing.renderDeferredShading(pctx, assets, timer, camera);
    }
    {
        DrawContext ctx = pctx.sub();
        ctx.setDepthTest(true);

        if (gbufferPipeline) {
            postProcessing.bindDepthBuffer();
        } else {
            ctx.setFramebuffer(postProcessing.getFramebuffer());
        }

        // Background sky plane
        skybox->draw(ctx, camera, assets, worldInfo.daytime, clouds);

        auto& linesShader = assets.require<Shader>("lines");
        linesShader.use();
        if (debug && hudVisible) {
            debugLines->render(
                ctx, camera, *lineBatch, linesShader, showChunkBorders
            );
        }
        linesShader.uniformMatrix("u_projview", projView);
        lineBatch->flush();

        skybox->bind();
        // Translucent blocks
        {
            auto sctx = ctx.sub();
            sctx.setCullFace(true);
            translucentShader.use();
            setupWorldShader(translucentShader, camera, settings, fogFactor);
            chunksRenderer->drawSortedMeshes(camera, translucentShader);
        }

        // Weather effects
        entityShader.use();
        setupWorldShader(entityShader, camera, settings, fogFactor);

        std::array<const WeatherPreset*, 2> weatherInstances {&weather.a, &weather.b};
        for (const auto& weather : weatherInstances) {
            float maxIntensity = weather->fall.maxIntensity;
            float zero = weather->fall.minOpacity;
            float one = weather->fall.maxOpacity;
            float t = (weather->intensity * (one - zero)) * maxIntensity + zero;
            entityShader.uniform1i("u_alphaClip", weather->fall.opaque);
            entityShader.uniform1f("u_opacity", weather->fall.opaque ? t * t : t);
            if (weather->intensity > 1.e-3f && !weather->fall.texture.empty()) {
                precipitation->render(camera, *weather);
            }
        }
        skybox->unbind();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    postProcessing.render(pctx, assets, timer, camera);
    
    if (player.currentCamera == player.fpCamera) {
        DrawContext ctx = pctx.sub();
        ctx.setDepthTest(true);
        ctx.setCullFace(true);

        // prepare modified HUD camera
        Camera hudcam = camera;
        hudcam.far = 10.0f;
        hudcam.setFov(0.9f);
        hudcam.position = {};
        
        hands->render(camera);

        display::clearDepth();
        setupWorldShader(entityShader, hudcam, engine.getSettings(), 0.0f);

        skybox->bind();
        modelBatch->render();
        modelBatch->setLightsOffset(glm::vec3());
        skybox->unbind();
    }
    renderBlockOverlay(pctx);

    glActiveTexture(GL_TEXTURE0);
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
    auto textureRegion = util::get_texture_region(
        assets, def.overlayTexture, "blocks:notfound"
    );
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

void WorldRenderer::clear() {
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
