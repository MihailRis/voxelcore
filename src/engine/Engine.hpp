#pragma once

#include "CoreParameters.hpp"
#include "PostRunnables.hpp"
#include "Time.hpp"
#include "delegates.hpp"
#include "settings.hpp"
#include "typedefs.hpp"
#include "util/ObjectsKeeper.hpp"

#include <memory>
#include <string>

class Assets;
class ContentControl;
class EngineController;
class EnginePaths;
class Input;
class Level;
class ResPaths;
class Screen;
class SettingsHandler;
class Window;
class WindowControl;
struct Project;

namespace gui {
    class GUI;
}

namespace cmd {
    class CommandsInterpreter;
}

namespace network {
    class Network;
}

namespace devtools {
    class Editor;
    class DebuggingServer;
}

class initialize_error : public std::runtime_error {
public:
    initialize_error(const std::string& message) : std::runtime_error(message) {}
};

using OnWorldOpen = std::function<void(std::unique_ptr<Level>, int64_t)>;

class Engine : public util::ObjectsKeeper {
    CoreParameters params;
    EngineSettings settings;
    std::unique_ptr<EnginePaths> paths;
    std::unique_ptr<Project> project;
    std::unique_ptr<SettingsHandler> settingsHandler;
    std::unique_ptr<Assets> assets;
    std::shared_ptr<Screen> screen;
    std::unique_ptr<ContentControl> content;
    std::unique_ptr<EngineController> controller;
    std::unique_ptr<cmd::CommandsInterpreter> cmd;
    std::unique_ptr<network::Network> network;
    std::unique_ptr<Window> window;
    std::unique_ptr<Input> input;
    std::unique_ptr<gui::GUI> gui;
    std::unique_ptr<devtools::Editor> editor;
    std::unique_ptr<devtools::DebuggingServer> debuggingServer;
    std::unique_ptr<WindowControl> windowControl;
    PostRunnables postRunnables;
    Time time;
    OnWorldOpen levelConsumer;
    bool quitSignal = false;
    
    void loadControls();
    void loadSettings();
    void saveSettings();
    void updateHotkeys();
    void loadAssets();
    void loadProject();

    void initializeClient();
    void onContentLoad();
public:
    Engine();
    ~Engine();

    static Engine& getInstance();

    void initialize(CoreParameters coreParameters);
    void close();

    static void terminate();

    /// @brief Start the engine
    void run();

    void postUpdate();

    void applicationTick();
    void updateFrontend();
    void renderFrame();
    void nextFrame(bool waitForRefresh);
    void startPauseLoop();
    
    /// @brief Set screen (scene).
    /// nullptr may be used to delete previous screen before creating new one,
    /// not-null value must be set before next frame
    /// @param screen nullable screen
    void setScreen(std::shared_ptr<Screen> screen);
    
    /// @brief Get active assets storage instance
    Assets* getAssets();

    /// @brief Get writeable engine settings structure instance
    EngineSettings& getSettings();

    /// @brief Get engine filesystem paths source
    EnginePaths& getPaths();

    /// @brief Get engine resource paths controller
    ResPaths& getResPaths();

    void onWorldOpen(std::unique_ptr<Level> level, int64_t localPlayer);
    void onWorldClosed();

    void quit();

    bool isQuitSignal() const;

    /// @brief Get current screen
    std::shared_ptr<Screen> getScreen();

    /// @brief Enqueue function call to the end of current frame in draw thread
    void postRunnable(const runnable& callback) {
        postRunnables.postRunnable(callback);
    }

    EngineController* getController();

    void setLevelConsumer(OnWorldOpen levelConsumer);

    SettingsHandler& getSettingsHandler();

    Time& getTime();

    const CoreParameters& getCoreParameters() const;

    bool isHeadless() const;

    ContentControl& getContentControl();

    gui::GUI& getGUI() {
        return *gui;
    }

    Input& getInput() {
        return *input;
    }

    Window& getWindow() {
        return *window;
    }

    network::Network& getNetwork() {
        return *network;
    }

    cmd::CommandsInterpreter& getCmd() {
        return *cmd;
    }

    devtools::Editor& getEditor() {
        return *editor;
    }

    const Project& getProject() {
        return *project;
    }

    devtools::DebuggingServer* getDebuggingServer() {
        return debuggingServer.get();
    }

    void detachDebugger();
};
