#include "Engine.hpp"

#ifndef GLEW_STATIC
#define GLEW_STATIC
#endif

#include "debug/Logger.hpp"
#include "assets/AssetsLoader.hpp"
#include "audio/audio.hpp"
#include "coders/GLSLExtension.hpp"
#include "coders/toml.hpp"
#include "coders/commons.hpp"
#include "devtools/Editor.hpp"
#include "devtools/Project.hpp"
#include "devtools/DebuggingServer.hpp"
#include "content/ContentControl.hpp"
#include "core_defs.hpp"
#include "io/io.hpp"
#include "io/settings_io.hpp"
#include "frontend/locale.hpp"
#include "frontend/menu.hpp"
#include "frontend/screens/Screen.hpp"
#include "graphics/render/ModelsGenerator.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "logic/EngineController.hpp"
#include "logic/CommandsInterpreter.hpp"
#include "logic/scripting/scripting.hpp"
#include "logic/scripting/scripting_hud.hpp"
#include "network/Network.hpp"
#include "util/platform.hpp"
#include "window/input.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"
#include "Mainloop.hpp"
#include "ServerMainloop.hpp"
#include "WindowControl.hpp"
#include "EnginePaths.hpp"

#include <iostream>
#include <assert.h>
#include <glm/glm.hpp>
#include <unordered_set>
#include <functional>
#include <utility>

static debug::Logger logger("engine");

Engine::Engine() = default;
Engine::~Engine() = default;

static std::unique_ptr<Engine> instance = nullptr;

Engine& Engine::getInstance() {
    if (!instance) {
        instance = std::make_unique<Engine>();
    }
    return *instance;
}

void Engine::onContentLoad() {
    editor->loadTools();
    langs::setup(langs::get_current(), paths->resPaths.collectRoots());
    
    if (isHeadless()) {
        return;
    }
    for (auto& pack : content->getAllContentPacks()) {
        auto configFolder = pack.folder / "config";
        auto bindsFile = configFolder / "bindings.toml";
        logger.info() << "loading bindings: " << bindsFile.string();
        if (io::is_regular_file(bindsFile)) {
            input->getBindings().read(
                toml::parse(
                    bindsFile.string(), io::read_string(bindsFile)
                ),
                BindType::BIND
            );
        }
    }
    loadAssets();
}

void Engine::initializeClient() {
    windowControl = std::make_unique<WindowControl>(*this);
    auto [window, input] = windowControl->initialize();

    this->window = std::move(window);
    this->input = std::move(input);

    loadControls();

    gui = std::make_unique<gui::GUI>(*this);
    if (ENGINE_DEBUG_BUILD) {
        menus::create_version_label(*gui);
    }
    keepAlive(settings.display.windowMode.observe(
        [this](int value) {
            WindowMode mode = static_cast<WindowMode>(value);
            if (mode != this->window->getMode()) {
                this->window->setMode(mode);
            }
        },
        true
    ));
    keepAlive(settings.debug.doTraceShaders.observe(
        [](bool value) {
            Shader::preprocessor->setTraceOutput(value);
        },
        true
    ));

    keepAlive(this->input->addKeyCallback(Keycode::ESCAPE, [this]() {
        auto& menu = *gui->getMenu();
        if (menu.hasOpenPage() && menu.back()) {
            return true;
        }
        return false;
    }));
}

void Engine::initialize(CoreParameters coreParameters) {
    params = std::move(coreParameters);
    settingsHandler = std::make_unique<SettingsHandler>(settings);

    logger.info() << "engine version: " << ENGINE_VERSION_STRING;
    if (params.headless) {
        logger.info() << "engine runs in headless mode";
    }
    if (params.projectFolder.empty()) {
        params.projectFolder = params.resFolder;
    }
    paths = std::make_unique<EnginePaths>(params);
    loadProject();

    editor = std::make_unique<devtools::Editor>(*this);
    cmd = std::make_unique<cmd::CommandsInterpreter>();
    network = network::Network::create(settings.network);

    if (!params.debugServerString.empty()) {
        try {
            debuggingServer = std::make_unique<devtools::DebuggingServer>(
                *this, params.debugServerString
            );
        } catch (const std::runtime_error& err) {
            throw initialize_error(
                "debugging server error: " + std::string(err.what())
            );
        }
    }
    loadSettings();

    controller = std::make_unique<EngineController>(*this);
    if (!params.headless) {
        initializeClient();
    }
    audio::initialize(!params.headless, settings.audio);

    if (settings.ui.language.get() == "auto") {
        settings.ui.language.set(
            langs::locale_by_envlocale(platform::detect_locale())
        );
    }
    content = std::make_unique<ContentControl>(*project, *paths, *input, [this]() {
        onContentLoad();
    });
    scripting::initialize(this);

    if (!isHeadless()) {
        gui->setPageLoader(scripting::create_page_loader());
    }
    keepAlive(settings.ui.language.observe([this](auto lang) {
        langs::setup(lang, paths->resPaths.collectRoots());
    }, true));

    keepAlive(settings.audio.inputDevice.observe([](auto name) {
        audio::set_input_device(name == "auto" ? "" : name);
    }, true));

    project->loadProjectStartScript();
    if (!params.headless) {
        project->loadProjectClientScript();
    }
}

void Engine::loadSettings() {
    io::path settings_file = EnginePaths::SETTINGS_FILE;
    if (io::is_regular_file(settings_file)) {
        logger.info() << "loading settings";
        std::string text = io::read_string(settings_file);
        try {
            toml::parse(*settingsHandler, settings_file.string(), text);
        } catch (const parsing_error& err) {
            logger.error() << err.errorLog();
            throw;
        }
    }
}

void Engine::loadControls() {
    io::path controls_file = EnginePaths::CONTROLS_FILE;
    if (io::is_regular_file(controls_file)) {
        logger.info() << "loading controls";
        std::string text = io::read_string(controls_file);
        input->getBindings().read(
            toml::parse(controls_file.string(), text), BindType::BIND
        );
    }
}

void Engine::updateHotkeys() {
    if (input->jpressed(Keycode::F2)) {
        windowControl->saveScreenshot();
    }
    if (input->pressed(Keycode::LEFT_CONTROL) && input->pressed(Keycode::F3) &&
        input->jpressed(Keycode::U)) {
        gui->toggleDebug();
    }
    if (input->jpressed(Keycode::F11)) {
        windowControl->toggleFullscreen();
    }
}

void Engine::run() {
    if (params.headless) {
        ServerMainloop(*this).run();
    } else {
        Mainloop(*this).run();
    }
}

void Engine::postUpdate() {
    network->update();
    postRunnables.run();
    scripting::process_post_runnables();

    if (debuggingServer) {
        debuggingServer->update();
    }
}

void Engine::detachDebugger() {
    debuggingServer.reset();
}

void Engine::applicationTick() {
    if (project->setupCoroutine && project->setupCoroutine->isActive()) {
        project->setupCoroutine->update();
    }
}

void Engine::updateFrontend() {
    double delta = time.getDelta();
    updateHotkeys();
    audio::update(delta);
    gui->act(delta, window->getSize());
    screen->update(delta);
    gui->postAct();
}

void Engine::nextFrame(bool waitForRefresh) {
    windowControl->nextFrame(waitForRefresh);
}

void Engine::startPauseLoop() {
    bool initialCursorLocked = false;
    if (!isHeadless()) {
        initialCursorLocked = input->isCursorLocked();
        if (initialCursorLocked) {
            input->toggleCursor();
        }
    }
    while (!isQuitSignal() && debuggingServer) {
        network->update();
        if (debuggingServer->update()) {
            break;
        }
        if (isHeadless()) {
            platform::sleep(1.0 / params.tps * 1000);
        } else {
            nextFrame(false);
        }
    }
    if (initialCursorLocked) {
        input->toggleCursor();
    }
}

void Engine::renderFrame() {
    screen->draw(time.getDelta());

    DrawContext ctx(nullptr, *window, nullptr);
    gui->draw(ctx, *assets);
}

void Engine::saveSettings() {
    logger.info() << "saving settings";
    io::write_string(EnginePaths::SETTINGS_FILE, toml::stringify(*settingsHandler));
    if (!params.headless) {
        logger.info() << "saving bindings";
        if (input) {
            io::write_string(
                EnginePaths::CONTROLS_FILE, input->getBindings().write()
            );
        }
    }
}

void Engine::close() {
    saveSettings();
    logger.info() << "shutting down";
    if (screen) {
        screen->onEngineShutdown();
        screen.reset();
    }
    content.reset();
    assets.reset();
    cmd.reset();
    if (gui) {
        gui.reset();
        logger.info() << "gui finished";
    }
    audio::close();
    debuggingServer.reset();
    network.reset();
    clearKeepedObjects();
    project.reset();
    scripting::close();
    logger.info() << "scripting finished";
    if (!params.headless) {
        window.reset();
        logger.info() << "window closed";
    }
    logger.info() << "engine finished";
}

void Engine::terminate() {
    instance->close();
    instance.reset();
}

EngineController* Engine::getController() {
    return controller.get();
}

void Engine::setLevelConsumer(OnWorldOpen levelConsumer) {
    this->levelConsumer = std::move(levelConsumer);
}

void Engine::loadAssets() {
    logger.info() << "loading assets";
    Shader::preprocessor->setPaths(&paths->resPaths);

    auto content = this->content->get();

    auto new_assets = std::make_unique<Assets>();
    AssetsLoader loader(*this, *new_assets, paths->resPaths);
    AssetsLoader::addDefaults(loader, content);

    // no need
    // correct log messages order is more useful
    // todo: before setting to true, check if GLSLExtension thread safe
    bool threading = false; // look at three upper lines
    if (threading) {
        auto task = loader.startTask([=](){});
        task->waitForEnd();
    } else {
        while (loader.hasNext()) {
            loader.loadNext();
        }
    }
    assets = std::move(new_assets);
    if (content) {
        ModelsGenerator::prepare(*content, *assets);
    }
    assets->setup();
    gui->onAssetsLoad(assets.get());
}

void Engine::loadProject() {
    io::path projectFile = "project:project.toml";
    project = std::make_unique<Project>();
    project->deserialize(io::read_object(projectFile));
    logger.info() << "loaded project " << util::quote(project->name);
}

void Engine::setScreen(std::shared_ptr<Screen> screen) {
    if (project->clientScript && this->screen) {
        project->clientScript->onScreenChange(this->screen->getName(), false);
    }
    // reset audio channels (stop all sources)
    audio::reset_channel(audio::get_channel_index("regular"));
    audio::reset_channel(audio::get_channel_index("ambient"));
    this->screen = std::move(screen);
    if (this->screen) {
        this->screen->onOpen();
    }
    if (project->clientScript && this->screen) {
        project->clientScript->onScreenChange(this->screen->getName(), true);
        window->setShouldRefresh();
    }
}

void Engine::onWorldOpen(std::unique_ptr<Level> level, int64_t localPlayer) {
    logger.info() << "world open";
    levelConsumer(std::move(level), localPlayer);
}

void Engine::onWorldClosed() {
    logger.info() << "world closed";
    levelConsumer(nullptr, -1);
}

void Engine::quit() {
    quitSignal = true;
    if (!isHeadless()) {
        window->setShouldClose(true);
    }
}

bool Engine::isQuitSignal() const {
    return quitSignal;
}

EngineSettings& Engine::getSettings() {
    return settings;
}

Assets* Engine::getAssets() {
    return assets.get();
}

EnginePaths& Engine::getPaths() {
    return *paths;
}

ResPaths& Engine::getResPaths() {
    return paths->resPaths;
}

std::shared_ptr<Screen> Engine::getScreen() {
    return screen;
}

SettingsHandler& Engine::getSettingsHandler() {
    return *settingsHandler;
}

Time& Engine::getTime() {
    return time;
}

const CoreParameters& Engine::getCoreParameters() const {
    return params;
}

bool Engine::isHeadless() const {
    return params.headless;
}

ContentControl& Engine::getContentControl() {
    return *content;
}
