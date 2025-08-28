#include "WorldRenderer.hpp"

#include <assert.h>

#include <algorithm>
#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

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
#include "logic/scripting/scripting_hud.hpp"
#include "maths/FrustumCulling.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "settings.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
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
#include "graphics/core/ShadowMap.hpp"
#include "graphics/core/GBuffer.hpp"
#include "BlockWrapsRenderer.hpp"
#include "ParticlesRenderer.hpp"
#include "PrecipitationRenderer.hpp"
#include "HandsRenderer.hpp"
#include "NamedSkeletons.hpp"
#include "TextsRenderer.hpp"
#include "ChunksRenderer.hpp"
#include "GuidesRenderer.hpp"
#include "ModelBatch.hpp"
#include "Skybox.hpp"
#include "Emitter.hpp"
#include "TextNote.hpp"

using namespace advanced_pipeline;

inline constexpr size_t BATCH3D_CAPACITY = 4096;
inline constexpr size_t MODEL_BATCH_CAPACITY = 20'000;
inline constexpr GLenum TEXTURE_MAIN = GL_TEXTURE0;
inline constexpr int MIN_SHADOW_MAP_RES = 512;

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
      guides(std::make_unique<GuidesRenderer>()),
      chunks(std::make_unique<ChunksRenderer>(
          &level,
          *player.chunks,
          assets,
          *frustumCulling,
          frontend.getContentGfxCache(),
          engine.getSettings()
      )),
      particles(std::make_unique<ParticlesRenderer>(
        assets, level, *player.chunks, &engine.getSettings().graphics
      )),
      texts(std::make_unique<TextsRenderer>(*batch3d, assets, *frustumCulling)),
      blockWraps(
          std::make_unique<BlockWrapsRenderer>(assets, level, *player.chunks)
      ),
      precipitation(std::make_unique<PrecipitationRenderer>(
          assets, level, *player.chunks, &engine.getSettings().graphics
      )) {
    auto& settings = engine.getSettings();
    level.events->listen(
        LevelEventType::CHUNK_HIDDEN,
        [this](LevelEventType, Chunk* chunk) { chunks->unload(chunk); }
    );
    auto assets = engine.getAssets();
    skybox = std::make_unique<Skybox>(
        settings.graphics.skyboxResolution.get(),
        assets->require<Shader>("skybox_gen")
    );

    const auto& content = level.content;
    skeletons = std::make_unique<NamedSkeletons>();
    const auto& skeletonConfig = content.requireSkeleton(
        content.getDefaults()["hand-skeleton"].asString()
    );
    hands = std::make_unique<HandsRenderer>(
        *assets, *modelBatch, skeletons->createSkeleton("hand", &skeletonConfig)
    );
}

WorldRenderer::~WorldRenderer() = default;

void WorldRenderer::setupWorldShader(
    Shader& shader,
    const Camera& camera,
    const EngineSettings& settings,
    float fogFactor
) {
    shader.use();
    shader.uniformMatrix("u_model", glm::mat4(1.0f));
    shader.uniformMatrix("u_proj", camera.getProjection());
    shader.uniformMatrix("u_view", camera.getView());
    shader.uniform1f("u_timer", timer);
    shader.uniform1f("u_gamma", settings.graphics.gamma.get());
    shader.uniform1f("u_fogFactor", fogFactor);
    shader.uniform1f("u_fogCurve", settings.graphics.fogCurve.get());
    shader.uniform1i("u_debugLights", lightsDebug);
    shader.uniform1i("u_debugNormals", false);
    shader.uniform1f("u_weatherFogOpacity", weather.fogOpacity());
    shader.uniform1f("u_weatherFogDencity", weather.fogDencity());
    shader.uniform1f("u_weatherFogCurve", weather.fogCurve());
    shader.uniform1f("u_dayTime", level.getWorld()->getInfo().daytime);
    shader.uniform2f("u_lightDir", skybox->getLightDir());
    shader.uniform3f("u_cameraPos", camera.position);
    shader.uniform1i("u_skybox", 1);
    shader.uniform1i("u_enableShadows", shadows);

    if (shadows) {
        const auto& worldInfo = level.getWorld()->getInfo();
        float cloudsIntensity = glm::max(worldInfo.fog, weather.clouds());
        shader.uniform1i("u_screen", 0);
        shader.uniformMatrix("u_shadowsMatrix[0]", shadowCamera.getProjView());
        shader.uniformMatrix("u_shadowsMatrix[1]", wideShadowCamera.getProjView());
        shader.uniform3f("u_sunDir", shadowCamera.front);
        shader.uniform1i("u_shadowsRes", shadowMap->getResolution());
        shader.uniform1f("u_shadowsOpacity", 1.0f - cloudsIntensity); // TODO: make it configurable
        shader.uniform1f("u_shadowsSoftness", 1.0f + cloudsIntensity * 4); // TODO: make it configurable

        glActiveTexture(GL_TEXTURE0 + TARGET_SHADOWS0);
        shader.uniform1i("u_shadows[0]", TARGET_SHADOWS0);
        glBindTexture(GL_TEXTURE_2D, shadowMap->getDepthMap());

        glActiveTexture(GL_TEXTURE0 + TARGET_SHADOWS1);
        shader.uniform1i("u_shadows[1]", TARGET_SHADOWS1);
        glBindTexture(GL_TEXTURE_2D, wideShadowMap->getDepthMap());

        glActiveTexture(TEXTURE_MAIN);
    }

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

void WorldRenderer::renderLevel(
    const DrawContext& ctx,
    const Camera& camera,
    const EngineSettings& settings,
    float delta,
    bool pause,
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
        delta,
        pause
    );
    modelBatch->render();
    particles->render(camera, delta * !pause);

    auto& shader = assets.require<Shader>("main");
    auto& linesShader = assets.require<Shader>("lines");

    setupWorldShader(shader, camera, settings, fogFactor);

    chunks->drawChunks(camera, shader);
    blockWraps->draw(ctx, player);

    if (hudVisible) {
        renderLines(camera, linesShader, ctx);
    }

    if (!pause) {
        scripting::on_frontend_render();
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

void WorldRenderer::generateShadowsMap(
    const Camera& camera,
    const DrawContext& pctx,
    ShadowMap& shadowMap,
    Camera& shadowCamera,
    float scale
) {
    auto& shadowsShader = assets.require<Shader>("shadows");

    auto world = level.getWorld();
    const auto& worldInfo = world->getInfo();

    const auto& settings = engine.getSettings();
    int resolution = shadowMap.getResolution();
    int quality = settings.graphics.shadowsQuality.get();
    float shadowMapScale = 0.32f / (1 << glm::max(0, quality)) * scale;
    float shadowMapSize = resolution * shadowMapScale;

    glm::vec3 basePos = glm::floor(camera.position / 4.0f) * 4.0f;
    glm::vec3 prevPos = shadowCamera.position;
    shadowCamera = Camera(
        glm::distance2(prevPos, basePos) > 25.0f ? basePos : prevPos,
        shadowMapSize
    );
    shadowCamera.near = 0.1f;
    shadowCamera.far = 1000.0f;
    shadowCamera.perspective = false;
    shadowCamera.setAspectRatio(1.0f);

    float t = worldInfo.daytime - 0.25f;
    if (t < 0.0f) {
        t += 1.0f;
    }
    t = fmod(t, 0.5f);

    float sunCycleStep = 1.0f / 500.0f;
    float sunAngle = glm::radians(
        90.0f -
        ((static_cast<int>(t / sunCycleStep)) * sunCycleStep + 0.25f) * 360.0f
    );
    float sunAltitude = glm::pi<float>() * 0.25f;
    shadowCamera.rotate(
        -glm::cos(sunAngle + glm::pi<float>() * 0.5f) * sunAltitude,
        sunAngle - glm::pi<float>() * 0.5f,
        glm::radians(0.0f)
    );

    shadowCamera.position -= shadowCamera.front * 500.0f;
    shadowCamera.position += shadowCamera.up * 0.0f;
    shadowCamera.position += camera.front * 0.0f;

    auto view = shadowCamera.getView();

    auto currentPos = shadowCamera.position;
    auto topRight = shadowCamera.right + shadowCamera.up;
    auto min = view * glm::vec4(currentPos - topRight * shadowMapSize * 0.5f, 1.0f);
    auto max = view * glm::vec4(currentPos + topRight * shadowMapSize * 0.5f, 1.0f);

    shadowCamera.setProjection(glm::ortho(min.x, max.x, min.y, max.y, 0.1f, 1000.0f));

    {
        auto sctx = pctx.sub();
        sctx.setDepthTest(true);
        sctx.setCullFace(true);
        sctx.setViewport({resolution, resolution});
        shadowMap.bind();
        setupWorldShader(shadowsShader, shadowCamera, settings, 0.0f);
        chunks->drawChunksShadowsPass(shadowCamera, shadowsShader, camera);
        shadowMap.unbind();
    }
}

void WorldRenderer::draw(
    const DrawContext& pctx,
    Camera& camera,
    bool hudVisible,
    bool pause,
    float uiDelta,
    PostProcessing& postProcessing
) {
    // TODO: REFACTOR WHOLE RENDER ENGINE
    float delta = uiDelta * !pause;
    timer += delta;
    weather.update(delta);

    auto world = level.getWorld();

    const auto& vp = pctx.getViewport();
    camera.setAspectRatio(vp.x / static_cast<float>(vp.y));

    auto& mainShader = assets.require<Shader>("main");
    auto& entityShader = assets.require<Shader>("entity");
    auto& translucentShader = assets.require<Shader>("translucent");
    auto& deferredShader = assets.require<PostEffect>("deferred_lighting").getShader();
    const auto& settings = engine.getSettings();

    gbufferPipeline = settings.graphics.advancedRender.get();
    int shadowsQuality = settings.graphics.shadowsQuality.get() * gbufferPipeline;
    int resolution = MIN_SHADOW_MAP_RES << shadowsQuality;
    if (shadowsQuality > 0 && !shadows) {
        shadowMap = std::make_unique<ShadowMap>(resolution);
        wideShadowMap = std::make_unique<ShadowMap>(resolution);
        shadows = true;
    } else if (shadowsQuality == 0 && shadows) {
        shadowMap.reset();
        wideShadowMap.reset();
        shadows = false;
    }

    CompileTimeShaderSettings currentSettings {
        gbufferPipeline,
        shadows,
        settings.graphics.ssao.get() && gbufferPipeline
    };
    if (
        prevCTShaderSettings.advancedRender != currentSettings.advancedRender ||
        prevCTShaderSettings.shadows != currentSettings.shadows ||
        prevCTShaderSettings.ssao != currentSettings.ssao
    ) {
        Shader::preprocessor->setDefined("ENABLE_SHADOWS", currentSettings.shadows);
        Shader::preprocessor->setDefined("ENABLE_SSAO", currentSettings.ssao);
        Shader::preprocessor->setDefined("ADVANCED_RENDER", currentSettings.advancedRender);
        mainShader.recompile();
        entityShader.recompile();
        deferredShader.recompile();
        translucentShader.recompile();
        prevCTShaderSettings = currentSettings;
    }

    if (shadows && shadowMap->getResolution() != resolution) {
        shadowMap = std::make_unique<ShadowMap>(resolution);
        wideShadowMap = std::make_unique<ShadowMap>(resolution);
    }

    const auto& worldInfo = world->getInfo();
    
    float clouds = weather.clouds();
    clouds = glm::max(worldInfo.fog, clouds);
    float mie = 1.0f + glm::max(worldInfo.fog, clouds * 0.5f) * 2.0f;

    skybox->refresh(pctx, worldInfo.daytime, mie, 4);

    chunks->update();

    static int frameid = 0;
    if (shadows) {
        if (frameid % 2 == 0) {
            generateShadowsMap(camera, pctx, *shadowMap, shadowCamera, 1.0f);
        } else {
            generateShadowsMap(camera, pctx, *wideShadowMap, wideShadowCamera, 3.0f);
        }
    }
    frameid++;

    auto& linesShader = assets.require<Shader>("lines");
    /* World render scope with diegetic HUD included */ {
        DrawContext wctx = pctx.sub();
        postProcessing.use(wctx, gbufferPipeline);

        display::clearDepth();

        /* Actually world render with depth buffer on */ {
            DrawContext ctx = wctx.sub();
            ctx.setDepthTest(true);
            ctx.setCullFace(true);
            renderLevel(ctx, camera, settings, uiDelta, pause, hudVisible);
            // Debug lines
            if (hudVisible) {
                if (debug) {
                    guides->renderDebugLines(
                        ctx, camera, *lineBatch, linesShader, showChunkBorders
                    );
                }
            }
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
            postProcessing.getFramebuffer()->bind();
        }
        // Drawing background sky plane
        skybox->draw(ctx, camera, assets, worldInfo.daytime, clouds);

        {
            auto sctx = ctx.sub();
            sctx.setCullFace(true);
            skybox->bind();
            translucentShader.use();
            setupWorldShader(translucentShader, camera, settings, fogFactor);
            chunks->drawSortedMeshes(camera, translucentShader);
            skybox->unbind();
        }

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
                precipitation->render(camera, pause ? 0.0f : delta, *weather);
            }
        }

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
        
        hands->renderHands(camera, delta);

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
    if (block && block->id) {
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
}

void WorldRenderer::clear() {
    chunks->clear();
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
