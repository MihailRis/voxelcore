#include "WindowControl.hpp"

#include "Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "devtools/Project.hpp"
#include "coders/imageio.hpp"
#include "window/Window.hpp"
#include "window/input.hpp"
#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "util/platform.hpp"

static debug::Logger logger("window-control");

namespace {
    static std::unique_ptr<ImageData> load_icon() {
        try {
            auto file = "res:textures/misc/icon.png";
            if (io::exists(file)) {
                return imageio::read(file);
            }
        } catch (const std::exception& err) {
            logger.error() << "could not load window icon: " << err.what();
        }
        return nullptr;
    }
}

WindowControl::WindowControl(Engine& engine) : engine(engine) {}

WindowControl::Result WindowControl::initialize() {
    const auto& project = engine.getProject();
    auto& settings = engine.getSettings();

    std::string title = project.title;
    if (!title.empty()) {
        title += " - ";
    }
    title += "VoxelCore v" +
                    std::to_string(ENGINE_VERSION_MAJOR) + "." +
                    std::to_string(ENGINE_VERSION_MINOR);
    if (ENGINE_DEBUG_BUILD) {
        title += " [debug]";
    }
    if (engine.getDebuggingServer()) {
        title = "[debugging] " + title;
    }

    auto [window, input] = Window::initialize(&settings.display, title);
    if (!window || !input){
        throw initialize_error("could not initialize window");
    }
    window->setFramerate(settings.display.framerate.get());
    if (auto icon = load_icon()) {
        icon->flipY();
        window->setIcon(icon.get());
    }

    return Result {std::move(window), std::move(input)};
}

void WindowControl::saveScreenshot() {
    auto& window = engine.getWindow();
    const auto& paths = engine.getPaths();

    auto image = window.takeScreenshot();
    image->flipY();
    io::path filename = paths.getNewScreenshotFile("png");
    imageio::write(filename.string(), image.get());
    logger.info() << "saved screenshot as " << filename.string();
}

void WindowControl::toggleFullscreen() {
    auto& settings = engine.getSettings();
    auto& windowMode = settings.display.windowMode;
    if (windowMode.get() != static_cast<int>(WindowMode::FULLSCREEN)) {
        windowMode.set(static_cast<int>(WindowMode::FULLSCREEN));
    } else {
        windowMode.set(static_cast<int>(WindowMode::WINDOWED));
    }
}

void WindowControl::nextFrame(bool waitForRefresh) {
    const auto& settings = engine.getSettings();
    auto& window = engine.getWindow();
    auto& input = engine.getInput();
    window.setFramerate(
        window.isIconified() && settings.display.limitFpsIconified.get()
            ? 20
            : settings.display.framerate.get()
    );
    window.swapBuffers();
    input.pollEvents(waitForRefresh && !window.checkShouldRefresh());
}
