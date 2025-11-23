#pragma once

#include "Panel.hpp"

namespace gui {
    class Label;

    class InputBindBox final : public Panel {
    protected:
        Binding& binding;
        glm::vec4 focusedColor {0.1f, 0.15f, 0.35f, 0.75f};
        std::shared_ptr<Label> label;
    public:
        explicit InputBindBox(
            GUI& gui, Binding& binding, glm::vec4 padding = glm::vec4(6.0f)
        );

        void drawBackground(
            const DrawContext& pctx, const Assets& assets
        ) override;

        void clicked(Mousecode button) override;
        void keyPressed(Keycode key) override;
        bool isFocuskeeper() const override {
            return true;
        }
    };
}
