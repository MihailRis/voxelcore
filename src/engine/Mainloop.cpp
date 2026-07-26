#include "Mainloop.hpp"

#include "debug/Logger.hpp"
#include "devtools/AppScriptsControl.hpp"
#include "devtools/Project.hpp"
#include "Engine.hpp"
#include "frontend/screens/LevelScreen.hpp"
#include "frontend/screens/MenuScreen.hpp"
#include "graphics/ui/elements/Container.hpp"
#include "graphics/ui/GUI.hpp"
#include "io/path.hpp"
#include "logic/scripting/scripting.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"

static debug::Logger logger("mainloop");

Mainloop::Mainloop(Engine& engine) : engine(engine) {
}

void Mainloop::run() {
    auto& time = engine.getTime();
    auto& window = engine.getWindow();
    auto& settings = engine.getSettings();
    const auto& coreParams = engine.getCoreParameters();

    engine.setLevelConsumer([this](auto level, int64_t localPlayer) {
        if (level == nullptr) {
            // destroy LevelScreen and run quit callbacks
            engine.setScreen(nullptr);
            // create and go to menu screen
            engine.setScreen(std::make_shared<MenuScreen>(engine));
        } else {
            engine.setScreen(std::make_shared<LevelScreen>(
                engine, std::move(level), localPlayer
            ));
        }
    });

    logger.info() << "starting menu screen";
    engine.setScreen(std::make_shared<MenuScreen>(engine));

    auto& appScripts = engine.getAppScripts();
    
    logger.info() << "main loop started";
    while (!window.isShouldClose()) {
        time.update(window.time());
        engine.applicationTick();
        engine.updateFrontend();

        if (!window.isIconified()) {
            engine.renderFrame();
        }
        engine.postUpdate();
        engine.nextFrame(
            settings.display.adaptiveFpsInMenu.get() &&
            dynamic_cast<const MenuScreen*>(engine.getScreen().get()) != nullptr
        );
        if (coreParams.testMode && appScripts.isFinished()) {
            logger.info() << "test finished";
            engine.quit();
        }
    }
    logger.info() << "main loop stopped";
}
