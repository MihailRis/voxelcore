#pragma once

#include "Panel.hpp"

namespace gui {
    class Label;

    class Button : public Panel {
    protected:
        std::shared_ptr<Label> label;
    public:
        Button(
            GUI& gui,
            const std::shared_ptr<UINode>& content,
            glm::vec4 padding = glm::vec4(2.0f)
        );

        Button(
            GUI& gui,
            const std::wstring& text,
            glm::vec4 padding,
            const onaction& action,
            glm::vec2 size = glm::vec2(-1)
        );

        virtual void drawBackground(
            const DrawContext& pctx, const Assets& assets
        ) override;

        virtual Align getTextAlign() const;
        virtual void setTextAlign(Align align);

        virtual void setText(std::wstring text);
        virtual std::wstring getText() const;

        virtual void refresh() override;
    };
}
