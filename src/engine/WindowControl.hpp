#pragma once

#include <memory>

class Window;
class Input;
class Engine;

class WindowControl {
public:
    struct Result {
        std::unique_ptr<Window> window;
        std::unique_ptr<Input> input;
    };
    WindowControl(Engine& engine);

    Result initialize();

    void nextFrame(bool waitForRefresh);

    void saveScreenshot();

    void toggleFullscreen();
private:
    Engine& engine;
};
