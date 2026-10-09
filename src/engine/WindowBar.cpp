#include "WindowBar.hpp"

#include "Engine.hpp"
#include "debug/Logger.hpp"
#include "graphics/core/Batch2D.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include "graphics/core/Shader.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/ui/GUI.hpp"
#include "window/Window.hpp"
#include "assets/Assets.hpp"

#include <algorithm>
#include <string_view>
#include <codecvt>
#include <locale>

namespace {
    std::wstring to_wide(const std::string& text) {
        std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
        return converter.from_bytes(text);
    }

    constexpr const char* BAR_FONT = "normal";
}

void draw_window_bar(Engine& engine) {
    auto& window = engine.getWindow();
    const int height = window.getDecorationHeight();
    if (height <= 0) {
        return;
    }
    auto* batch = engine.getGUI().getBatch2D();
    auto font = engine.requireAssets().getShared<Font>(BAR_FONT);
    if (batch == nullptr || font == nullptr) {
        return;
    }
    const auto size = window.getSize();
    const float barHeight = static_cast<float>(height);

    glViewport(0, 0, static_cast<GLsizei>(size.x), static_cast<GLsizei>(size.y));
    if (auto shader = engine.requireAssets().get<Shader>("ui"); shader != nullptr) {
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
    const std::wstring title = to_wide(window.getTitle());
    const float available = std::max(0.0f, titleRight - titleLeft);
    size_t titleLength = title.size();
    while (titleLength > 0 && font->calcWidth(title, titleLength) > available) {
        titleLength--;
    }
    if (titleLength > 0) {
        const float textWidth = static_cast<float>(
            font->calcWidth(title, titleLength)
        );
        const float centered = (static_cast<float>(size.x) - textWidth) * 0.5f;
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

    const float thickness = 2.0f;
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
        const char* iconName =
            item.button == DecorationButton::CLOSE
                ? "gui/cross"
                : item.button == DecorationButton::MINIMIZE
                      ? "gui/minimize_w"
                      : window.isMaximized() ? "gui/restore_w"
                                             : "gui/maximize_w";
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
            continue;
        }
        if (item.button == DecorationButton::CLOSE) {
            for (int step = 0; step < 8; step++) {
                batch->rect(cx - 5.0f + step, cy - 5.0f + step, thickness, thickness);
                batch->rect(cx + 3.0f - step, cy - 5.0f + step, thickness, thickness);
            }
        } else if (item.button == DecorationButton::MINIMIZE) {
            batch->rect(cx - 6.0f, cy - 1.0f, 12.0f, thickness);
        } else if (!window.isMaximized()) {
            batch->rect(cx - 6.0f, cy - 6.0f, 12.0f, thickness);
            batch->rect(cx - 6.0f, cy + 4.0f, 12.0f, thickness);
            batch->rect(cx - 6.0f, cy - 6.0f, thickness, 12.0f);
            batch->rect(cx + 4.0f, cy - 6.0f, thickness, 12.0f);
        } else {
            batch->rect(cx - 7.0f, cy + 1.0f, 11.0f, thickness);
            batch->rect(cx - 7.0f, cy + 1.0f, thickness, 6.0f);
            batch->rect(cx + 2.0f, cy + 1.0f, thickness, 6.0f);
            batch->rect(cx - 7.0f, cy + 5.0f, 11.0f, thickness);
            batch->rect(cx - 2.0f, cy - 5.0f, 9.0f, thickness);
            batch->rect(cx - 2.0f, cy - 5.0f, thickness, 6.0f);
            batch->rect(cx + 5.0f, cy - 5.0f, thickness, 6.0f);
        }
    }
    batch->flush();
}
