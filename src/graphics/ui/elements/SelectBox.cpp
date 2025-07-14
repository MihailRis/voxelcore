#include "SelectBox.hpp"

#include "Label.hpp"
#include "assets/Assets.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Panel.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"

using namespace gui;

SelectBox::SelectBox(
    GUI& gui,
    std::vector<Option>&& options,
    Option selected,
    int contentWidth,
    const glm::vec4& padding
)
    : Button(gui, selected.text, padding, nullptr, glm::vec2(contentWidth, -1)),
      options(std::move(options)) {

    listenAction([this](GUI& gui) {
        auto panel = std::make_shared<Panel>(gui, getSize());
        panel->setPos(calcPos() + glm::vec2(0, size.y));
        for (const auto& option : this->options) {
            auto button = std::make_shared<Button>(
                gui, option.text, glm::vec4(10.0f), nullptr, glm::vec2(-1.0f)
            );
            button->listenFocus([this, option](GUI& gui) {
                setSelected(option);
                changeCallbacks.notify(gui, option.value);
            });
            panel->add(button);
        }
        panel->setZIndex(GUI::CONTEXT_MENU_ZINDEX);
        gui.setFocus(panel);
        panel->listenDefocus([panel=panel.get()](GUI& gui) {
            gui.remove(panel);
        });
        gui.add(panel);
    });
}

void SelectBox::listenChange(onstringchange&& callback) {
    changeCallbacks.listen(std::move(callback));
}

void SelectBox::setSelected(const Option& selected) {
    this->selected = selected;
    this->label->setText(selected.text);
}

const SelectBox::Option& SelectBox::getSelected() const {
    return selected;
}

const std::vector<SelectBox::Option>& SelectBox::getOptions() const {
    return options;
}

void SelectBox::setOptions(std::vector<Option>&& options) {
    this->options = std::move(options);
}

void SelectBox::drawBackground(const DrawContext& pctx, const Assets&) {
    glm::vec2 pos = calcPos();
    auto batch = pctx.getBatch2D();
    batch->untexture();
    batch->setColor(calcColor());
    batch->rect(pos.x, pos.y, size.x, size.y);
    batch->setColor({1.0f, 1.0f, 1.0f, 0.333f});

    int paddingRight = padding.w;
    int widthHalf = 8;
    int heightHalf = 4;
    batch->triangle(
        pos.x + size.x - paddingRight - widthHalf * 2,
        pos.y + size.y / 2.0f - heightHalf,
        pos.x + size.x - paddingRight,
        pos.y + size.y / 2.0f - heightHalf,
        pos.x + size.x - paddingRight - widthHalf,
        pos.y + size.y / 2.0f + heightHalf
    );
}
