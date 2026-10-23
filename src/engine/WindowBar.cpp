#include "WindowBar.hpp"

#include "Engine.hpp"
#include "graphics/core/Batch2D.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include "graphics/core/Shader.hpp"
#include "constants.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/ui/GUI.hpp"
#include "window/Window.hpp"
#include "assets/Assets.hpp"
#include "util/stringutil.hpp"

#include <algorithm>
#include <string_view>

/// @brief Draws the window bar the backend cannot draw itself.
///
/// The bar lives in window coordinates, so it sets the shader and projection
/// itself instead of depending on the state the GUI left behind.
void draw_window_bar(Engine& engine) {
    auto& window = engine.getWindow();
    const int height = window.getDecorationHeight();
    if (height <= 0) {
        return;
    }
    auto* batch = engine.getGUI().getBatch2D();
    auto font = engine.requireAssets().getShared<Font>(FONT_DEFAULT);
    const auto size = window.getSize();
    const float barHeight = static_cast<float>(height);

    glViewport(
        0, 0, static_cast<GLsizei>(size.x), static_cast<GLsizei>(size.y)
    );
    auto shader = engine.requireAssets().get<Shader>("ui");
    if (shader != nullptr) {
        shader->use();
        shader->uniformMatrix(
            "u_projview",
            glm::ortho(
                0.0f,
                static_cast<float>(size.x),
                static_cast<float>(size.y),
                0.0f,
                -1.0f,
                1.0f
            )
        );
    }

    batch->begin();
    batch->setColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.82f));
    batch->rect(0.0f, 0.0f, size.x, barHeight);

    const auto& buttons = window.getDecorationButtons();
    const int hovered = window.getDecorationHoveredButton();

    float titleLeft = static_cast<float>(barHeight * 0.4f);
    float titleRight = size.x - 8.0f;
    for (const auto& item : buttons) {
        if (item.x + item.width * 0.5f < size.x * 0.5f) {
            titleLeft = std::max(titleLeft, item.x + item.width + 8.0f);
        } else {
            titleRight = std::min(titleRight, item.x - 8.0f);
        }
    }
    // the bar and its controls do not need the font, only the title does
    if (font != nullptr) {
        const std::wstring title = util::str2wstr_utf8(window.getTitle());
        const float available = std::max(0.0f, titleRight - titleLeft);
        size_t titleLength = title.size();
        while (titleLength > 0 &&
               font->calcWidth(title, titleLength) > available) {
            titleLength--;
        }
        if (titleLength > 0) {
            const float textWidth = static_cast<float>(
                font->calcWidth(title, titleLength)
            );
            const float centered =
                (static_cast<float>(size.x) - textWidth) * 0.5f;
            const float titleX = std::clamp(
                centered,
                titleLeft,
                std::max(titleLeft, titleRight - textWidth)
            );
            batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, 0.85f));
            font->draw(
                *batch, std::wstring_view(title).substr(0, titleLength),
                static_cast<int>(titleX),
                (height - font->getLineHeight()) / 2,
                nullptr, 0
            );
            batch->untexture();
        }
    }

    for (size_t i = 0; i < buttons.size(); i++) {
        const auto& item = buttons[i];
        const float x = item.x;
        const float cx = x + item.width * 0.5f;
        const float cy = barHeight * 0.5f;
        const bool isHovered = hovered == static_cast<int>(i);
        if (isHovered) {
            batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, 0.16f));
            const float button = item.width;
            batch->rect(
                item.x, (barHeight - button) * 0.5f, item.width, button
            );
        }
        const float ink = isHovered ? 1.0f : 0.8f;
        batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, ink));
        const char* iconName = nullptr;
        switch (item.button) {
            case DecorationButton::CLOSE:
                iconName = "gui/cross";
                break;
            case DecorationButton::MINIMIZE:
                iconName = "gui/minimize_w";
                break;
            case DecorationButton::MAXIMIZE:
            default:
                iconName = window.isMaximized() ? "gui/restore_w"
                                                : "gui/maximize_w";
                break;
        }
        auto icon = engine.requireAssets().getShared<Texture>(iconName);
        if (icon != nullptr) {
            const float iconSize = std::min(
                16.0f, static_cast<float>(icon->getWidth())
            );
            batch->texture(icon.get());
            batch->rect(
                cx - iconSize * 0.5f, cy - iconSize * 0.5f, iconSize,
                iconSize, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, ink
            );
            batch->untexture();
        }

    }
    batch->flush();
}
