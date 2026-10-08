#include "WindowBar.hpp"

#include "Engine.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/ui/GUI.hpp"
#include "window/Window.hpp"
#include "assets/Assets.hpp"

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

    batch->begin();
    batch->setColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.82f));
    batch->rect(0.0f, 0.0f, size.x, barHeight);
    batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, 0.10f));
    batch->rect(0.0f, barHeight - 1.0f, size.x, 1.0f);

    const std::wstring title = to_wide(window.getTitle());
    const int textX = static_cast<int>(barHeight * 0.4f);
    const int textY = (height - font->getLineHeight()) / 2;
    batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, 0.85f));
    font->draw(*batch, title, textX, textY, nullptr, 0);
    batch->untexture();

    const float button = barHeight;
    const int hovered = window.getDecorationHoveredButton();
    for (int i = 0; i < 3; i++) {
        const float x = size.x - button * (i + 1);
        const float cx = x + button * 0.5f;
        const float cy = barHeight * 0.5f;
        if (hovered == i) {
            batch->setColor(
                i == 0 ? glm::vec4(0.62f, 0.13f, 0.09f, 0.9f)
                       : glm::vec4(1.0f, 1.0f, 1.0f, 0.16f)
            );
            batch->rect(x, 0.0f, button, barHeight - 1.0f);
        }
        const float ink = hovered == i ? 1.0f : 0.8f;
        const float thickness = 2.0f;
        batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, ink));
        if (i == 0) {
            for (int s = 0; s < 8; s++) {
                batch->rect(cx - 5.0f + s, cy - 5.0f + s, thickness, thickness);
                batch->rect(cx + 3.0f - s, cy - 5.0f + s, thickness, thickness);
            }
        } else if (i == 1 && !window.isMaximized()) {
            batch->rect(cx - 6.0f, cy - 6.0f, 12.0f, thickness);
            batch->rect(cx - 6.0f, cy + 4.0f, 12.0f, thickness);
            batch->rect(cx - 6.0f, cy - 6.0f, thickness, 12.0f);
            batch->rect(cx + 4.0f, cy - 6.0f, thickness, 12.0f);
        } else if (i == 1) {
            batch->rect(cx - 7.0f, cy + 1.0f, 11.0f, thickness);
            batch->rect(cx - 7.0f, cy + 1.0f, thickness, 6.0f);
            batch->rect(cx + 2.0f, cy + 1.0f, thickness, 6.0f);
            batch->rect(cx - 7.0f, cy + 5.0f, 11.0f, thickness);
            batch->rect(cx - 2.0f, cy - 5.0f, 9.0f, thickness);
            batch->rect(cx - 2.0f, cy - 5.0f, thickness, 6.0f);
            batch->rect(cx + 5.0f, cy - 5.0f, thickness, 6.0f);
        } else {
            batch->rect(cx - 6.0f, cy - 1.0f, 12.0f, thickness);
        }
    }
    batch->flush();
}
