#include "LevelScreen.hpp"

#include "audio/audio.hpp"
#include "coders/imageio.hpp"
#include "content/Content.hpp"
#include "core_defs.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "assets/Assets.hpp"
#include "frontend/ContentGfxCache.hpp"
#include "frontend/LevelFrontend.hpp"
#include "frontend/hud.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/PostProcessing.hpp"
#include "graphics/render/Decorator.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/core/TextureAnimation.hpp"
#include "io/io.hpp"
#include "logic/LevelController.hpp"
#include "logic/PlayerController.hpp"
#include "logic/scripting/scripting.hpp"
#include "logic/scripting/scripting_hud.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Players.hpp"
#include "physics/Hitbox.hpp"
#include "util/stringutil.hpp"
#include "voxels/Chunks.hpp"
#include "window/Camera.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"

static debug::Logger logger("level-screen");

inline const io::path CLIENT_FILE = "world:client/environment.json";

LevelScreen::LevelScreen(
    Engine& engine, std::unique_ptr<Level> levelPtr, int64_t localPlayer
)
    : Screen(engine),
      world(*levelPtr->getWorld()),
      postProcessing(std::make_unique<PostProcessing>(
          levelPtr->content.getIndices(ResourceType::POST_EFFECT_SLOT).size()
      )),
      gui(engine.getGUI()),
      input(engine.getInput()) {
    Level* level = levelPtr.get();

    auto& settings = engine.getSettings();
    auto& assets = *engine.getAssets();
    auto menu = engine.getGUI().getMenu();
    menu->reset();

    auto player = level->players->get(localPlayer);
    assert(player != nullptr);

    controller =
        std::make_unique<LevelController>(&engine, std::move(levelPtr), player);
    playerController = std::make_unique<PlayerController>(
        settings, *level, *player, *controller->getBlocksController()
    );

    frontend = std::make_unique<LevelFrontend>(
        engine, player, controller.get(), settings
    );
    renderer = std::make_unique<WorldRenderer>(
        engine, *frontend, *player
    );
    hud = std::make_unique<Hud>(engine, *frontend, *player);

    decorator = std::make_unique<Decorator>(
        engine, *controller, *renderer, assets, *player
    );

    keepAlive(settings.graphics.backlight.observe([=](bool) {
        player->chunks->saveAndClear();
        renderer->clear();
    }));
    keepAlive(settings.graphics.denseRender.observe([=](bool) {
        player->chunks->saveAndClear();
        renderer->clear();
        frontend->getContentGfxCache().refresh();
    }));
    keepAlive(settings.camera.fov.observe([=](double value) {
        player->fpCamera->setFov(glm::radians(value));
    }));
    keepAlive(input.addCallback(BIND_CHUNKS_RELOAD, [=]() {
        player->chunks->saveAndClear();
        renderer->clear();
        return false;
    }));

    animator = std::make_unique<TextureAnimator>();
    animator->addAnimations(assets.getAnimations());

    loadDecorations();
}

LevelScreen::~LevelScreen() {
    if (!controller->getLevel()->getWorld()->isNameless()) {
        saveDecorations();
        saveWorldPreview();
    }
    scripting::on_frontend_close();
    // unblock all bindings
    input.getBindings().enableAll();
    controller->onWorldQuit();
    engine.getPaths().setCurrentWorldFolder("");
}

void LevelScreen::onOpen() {
    initializeContent();
}

void LevelScreen::initializeContent() {
    auto& content = controller->getLevel()->content;
    for (auto& entry : content.getPacks()) {
        logger.info() << "initializing pack '" << entry.first << "'";
        initializePack(entry.second.get());
    }
    scripting::on_frontend_init(
        hud.get(), renderer.get(), postProcessing.get()
    );
}

void LevelScreen::initializePack(ContentPackRuntime* pack) {
    const ContentPack& info = pack->getInfo();
    io::path scriptFile = info.folder / "scripts/hud.lua";
    if (io::is_regular_file(scriptFile)) {
        scripting::load_hud_script(
            pack->getEnvironment(),
            info.id,
            scriptFile,
            pack->getId() + ":scripts/hud.lua"
        );
    }
}

void LevelScreen::loadDecorations() {
    if (!io::exists(CLIENT_FILE)) {
        return;
    }
    auto data = io::read_object(CLIENT_FILE);
    if (data.has("weather")) {
        renderer->getWeather().deserialize(data["weather"]);
    }
}

void LevelScreen::saveDecorations() {
    io::create_directory("world:client");

    auto data = dv::object();
    data["weather"] = renderer->getWeather().serialize();
    io::write_json(CLIENT_FILE, data, true);
}

void LevelScreen::saveWorldPreview() {
    try {
        logger.info() << "saving world preview";
        auto player = playerController->getPlayer();
        auto& settings = engine.getSettings();
        int previewSize = settings.ui.worldPreviewSize.get();

        // camera special copy for world preview
        Camera camera = *player->fpCamera;
        camera.setFov(glm::radians(70.0f));

        DrawContext pctx(nullptr, engine.getWindow(), batch.get());

        DrawContext ctx(&pctx, engine.getWindow(), batch.get());
        ctx.setViewport(
            {static_cast<uint>(previewSize * 1.5),
             static_cast<uint>(previewSize)}
        );

        renderer->draw(ctx, camera, false, true, 0.0f, *postProcessing);
        auto image = postProcessing->toImage();
        image->flipY();
        imageio::write("world:preview.png", image.get());
    } catch (const std::exception& err) {
        logger.error() << err.what();
    }
}

void LevelScreen::updateHotkeys() {
    auto& settings = engine.getSettings();

    if (input.jpressed(Keycode::F1)) {
        hudVisible = !hudVisible;
    }
    if (!input.pressed(Keycode::LEFT_CONTROL)) {
        if (input.jpressed(Keycode::F3)) {
            debug = !debug;
            hud->setDebug(debug);
            renderer->setDebug(debug);
        }
    } else if (input.pressed(Keycode::F3)) {
        if (input.jpressed(Keycode::L)) {
            renderer->toggleLightsDebug();
        } else if (input.jpressed(Keycode::O)) {
            settings.graphics.frustumCulling.toggle();
        }
    }
}

void LevelScreen::updateAudio() {
    auto player = playerController->getPlayer();
    auto camera = player->currentCamera;
    bool paused = hud->isPause();

    audio::get_channel("regular")->setPaused(paused);
    audio::get_channel("ambient")->setPaused(paused);
    glm::vec3 velocity {};
    if (auto hitbox = player->getHitbox()) {
        velocity = hitbox->velocity;
    }
    audio::set_listener(
        camera->position, velocity, camera->dir, glm::vec3(0, 1, 0)
    );
}

void LevelScreen::update(float delta) {
    auto& gui = engine.getGUI();
    
    if (!gui.isFocusCaught()) {
        updateHotkeys();
    }
    updateAudio();
    
    auto menu = gui.getMenu();
    bool inputLocked =
        menu->hasOpenPage() || hud->isInventoryOpen() || gui.isFocusCaught();
    bool paused = hud->isPause();
    if (!paused) {
        world.updateTimers(delta);
        animator->update(delta);
        playerController->update(delta, inputLocked ? nullptr : &engine.getInput());
    }
    controller->update(glm::min(delta, 0.2f), paused);
    playerController->postUpdate(
        delta,
        engine.getWindow().getSize().y,
        inputLocked ? nullptr : &engine.getInput(),
        paused
    );

    hud->update(hudVisible);

    const auto& weather = renderer->getWeather();
    const auto& player = *playerController->getPlayer();
    const auto& camera = *player.currentCamera;
    decorator->update(paused ? 0.0f : delta, camera, weather);
}

void LevelScreen::draw(float delta) {
    auto camera = playerController->getPlayer()->currentCamera;

    DrawContext ctx(nullptr, engine.getWindow(), batch.get());

    if (!hud->isPause()) {
        scripting::on_entities_render(engine.getTime().getDelta());
    }
    renderer->draw(
        ctx, *camera, hudVisible, hud->isPause(), delta, *postProcessing
    );

    if (hudVisible) {
        hud->draw(ctx);
    }
}

void LevelScreen::onEngineShutdown() {
    if (hud->isInventoryOpen()) {
        hud->closeInventory();
    }
    controller->saveWorld();
}
