#include "Button.hpp"

#include <utility>

#include "Label.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"

using namespace gui;

Button::Button(
    GUI& gui, const std::shared_ptr<UINode>& content, glm::vec4 padding
)
    : Panel(gui, glm::vec2(), padding, 0) {
    glm::vec4 margin = getMargin();
    setSize(
        content->getSize() +
        glm::vec2(
            padding[0] + padding[2] + margin[0] + margin[2],
            padding[1] + padding[3] + margin[1] + margin[3]
        )
    );
    add(content);
    setScrollable(false);
    setHoverColor(glm::vec4(0.05f, 0.1f, 0.15f, 0.75f));
    setPressedColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.95f));
    content->setInteractive(false);
}

Button::Button(
    GUI& gui,
    const std::wstring& text,
    glm::vec4 padding,
    const onaction& action,
    glm::vec2 size
)
    : Panel(gui, size, padding, 0.0f) {
    if (size.x < 0.0f || size.y < 0.0f) {
        setContentSize({text.length() * 8, 16});
    } else {
        setSize(size);
    }

    if (action) {
        listenAction(action);
    }
    setScrollable(false);

    label = std::make_shared<Label>(gui, text);
    label->setAlign(Align::center);
    label->setSize(getContentSize());
    label->setInteractive(false);
    add(label);
    
    setHoverColor({0.05f, 0.1f, 0.15f, 0.75f});
    setPressedColor({0.0f, 0.0f, 0.0f, 0.95f});
}

void Button::setText(std::wstring text) {
    if (label) {
        label->setText(std::move(text));
    }
}

std::wstring Button::getText() const {
    if (label) {
        return label->getText();
    }
    return L"";
}

void Button::refresh() {
    Panel::refresh();
    if (label) {
        label->setSize(getContentSize());
    }
}

void Button::drawBackground(const DrawContext& pctx, const Assets&) {
    glm::vec2 pos = calcPos();
    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);
    batch->setColor(calcColor());
    batch->rect(pos.x, pos.y, size.x, size.y);
}

void Button::setTextAlign(Align align) {
    if (label) {
        label->setAlign(align);
        refresh();
    }
}

Align Button::getTextAlign() const {
    if (label) {
        return label->getAlign();
    }
    return Align::left;
}
