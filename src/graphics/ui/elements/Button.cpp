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
    : Panel(gui, size, padding, 0) {
    if (size.y < 0.0f) {
        size = glm::vec2(
            glm::max(padding.x + padding.z + text.length() * 8, size.x),
            glm::max(padding.y + padding.w + 16, size.y)
        );
    }
    setSize(size);

    if (action) {
        listenAction(action);
    }
    setScrollable(false);

    label = std::make_shared<Label>(gui, text);
    label->setAlign(Align::center);
    label->setSize(
        size - glm::vec2(padding.z + padding.x, padding.w + padding.y)
    );
    label->setInteractive(false);
    add(label);
    setHoverColor(glm::vec4(0.05f, 0.1f, 0.15f, 0.75f));
    setPressedColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.95f));
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

Button* Button::textSupplier(wstringsupplier supplier) {
    if (label) {
        label->textSupplier(std::move(supplier));
    }
    return this;
}

void Button::refresh() {
    Panel::refresh();
    if (label) {
        label->setSize(
            size - glm::vec2(padding.z + padding.x, padding.w + padding.y)
        );
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
