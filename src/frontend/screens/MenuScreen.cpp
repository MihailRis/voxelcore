#include "MenuScreen.hpp"

#include "content/ContentControl.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "assets/Assets.hpp"
#include "maths/UVRegion.hpp"
#include "window/Window.hpp"
#include "window/Camera.hpp"
#include "engine/Engine.hpp"

MenuScreen::MenuScreen(Engine& engine) : Screen(engine) {
    engine.getContentControl().resetContent();
    
    auto menu = engine.getGUI().getMenu();
    menu->reset();
    menu->setPage("main");

    uicamera =
        std::make_unique<Camera>(glm::vec3(), engine.getWindow().getSize().y);
    uicamera->perspective = false;
    uicamera->near = -1.0f;
    uicamera->far = 1.0f;
    uicamera->flipped = true;
}

MenuScreen::~MenuScreen() = default;

void MenuScreen::update(float delta) {
}

void MenuScreen::draw(float delta) {
    auto assets = engine.getAssets();

    display::clear();
    display::setBgColor(glm::vec3(0.2f));

    const auto& size = engine.getWindow().getSize();
    uint width = size.x;
    uint height = size.y;

    uicamera->setFov(height);
    uicamera->setAspectRatio(width / static_cast<float>(height));
    auto uishader = assets->get<Shader>("ui");
    uishader->use();
    uishader->uniformMatrix("u_projview", uicamera->getProjView());

    auto bg = assets->get<Texture>("gui/menubg");
    batch->begin();
    batch->texture(bg);
    batch->rect(
        0, 0, 
        width, height, 0, 0, 0, 
        UVRegion(0, 0, width / bg->getWidth(), height / bg->getHeight()), 
        false, false, glm::vec4(1.0f)
    );
    batch->flush();
}
