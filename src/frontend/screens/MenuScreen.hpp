#pragma once

#include "Screen.hpp"

#include <memory>

class Camera;
class Engine;

class MenuScreen : public Screen {
    std::unique_ptr<Camera> uicamera;
public:
    MenuScreen(Engine& engine);
    ~MenuScreen();

    void onOpen() override;

    int getContentInset(const Window&) const override {
        return 0;
    }

    void update(float delta) override;
    void draw(float delta) override;

    const char* getName() const override {
        return "menu";
    }
};
