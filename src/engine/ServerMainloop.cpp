#include "ServerMainloop.hpp"

#include "Engine.hpp"
#include "devtools/AppScriptsControl.hpp"
#include "logic/LevelController.hpp"
#include "interfaces/Process.hpp"
#include "debug/Logger.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "util/platform.hpp"
#include "devtools/Project.hpp"

#include <chrono>

using namespace std::chrono;

static debug::Logger logger("mainloop");

ServerMainloop::ServerMainloop(Engine& engine) : engine(engine) {
}

ServerMainloop::~ServerMainloop() = default;

void ServerMainloop::run() {
    const auto& coreParams = engine.getCoreParameters();
    auto& time = engine.getTime();

    if (coreParams.scriptFile.empty()) {
        logger.info() << "nothing to do";
        return;
    }
    engine.setLevelConsumer([this](auto level, auto) {
        setLevel(std::move(level));
    });

    double targetDelta = 1.0 / static_cast<double>(coreParams.tps);
    double delta = targetDelta;
    const auto tickDuration =
        duration_cast<steady_clock::duration>(duration<double>(targetDelta));
    auto startupTime = steady_clock::now();
    auto nextTick = startupTime;

    while (!engine.isQuitSignal() && !engine.getAppScripts().isFinished()) {
        if (coreParams.testMode) {
            time.step(delta);
        } else {
            time.update(duration<double>(nextTick - startupTime).count());
            delta = time.getDelta();
        }
        if (controller) {
            controller->getLevel()->getWorld().updateTimers(delta);
            controller->update(glm::min(delta, 0.2), false);
        }
        engine.applicationTick();
        engine.postUpdate();

        if (!coreParams.testMode) {
            nextTick += tickDuration;
            auto now = steady_clock::now();
            if (nextTick > now) {
                platform::sleep(
                    duration_cast<milliseconds>(nextTick - now).count()
                );
            } else if (now - nextTick > tickDuration * 5) {
                nextTick = now;
            }
        }
    }
    logger.info() << "script finished";
}

void ServerMainloop::setLevel(std::unique_ptr<Level> level) {
    if (level == nullptr) {
        controller->onWorldQuit();
        controller = nullptr;
    } else {
        controller = std::make_unique<LevelController>(
            engine, std::move(level), nullptr
        );
    }
}
