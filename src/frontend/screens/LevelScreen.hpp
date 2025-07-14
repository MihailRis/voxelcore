#pragma once

#include "Screen.hpp"

#include <memory>
#include <cstdint>

class Engine;
class LevelFrontend;
class Hud;
class LevelController;
class PlayerController;
class WorldRenderer;
class TextureAnimator;
class PostProcessing;
class ContentPackRuntime;
class Decorator;
class Level;
class World;
class Input;

namespace gui {
    class GUI;
}

class LevelScreen : public Screen {
    World& world;
    std::unique_ptr<LevelFrontend> frontend;
    std::unique_ptr<LevelController> controller;
    std::unique_ptr<PlayerController> playerController;
    std::unique_ptr<WorldRenderer> renderer;
    std::unique_ptr<TextureAnimator> animator;
    std::unique_ptr<PostProcessing> postProcessing;
    std::unique_ptr<Decorator> decorator;
    std::unique_ptr<Hud> hud;
    gui::GUI& gui;
    Input& input;

    void saveWorldPreview();

    bool hudVisible = true;
    bool debug = false;
    void updateHotkeys();
    void initializeContent();
    void initializePack(ContentPackRuntime* pack);

    void loadDecorations();
    void saveDecorations();
    void updateAudio();
public:
    LevelScreen(
        Engine& engine, std::unique_ptr<Level> level, int64_t localPlayer
    );
    ~LevelScreen();

    void update(float delta) override;
    void draw(float delta) override;

    void onEngineShutdown() override;
};
