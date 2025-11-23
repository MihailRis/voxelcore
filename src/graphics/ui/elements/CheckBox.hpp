#pragma once

#include <utility>

#include "Label.hpp"
#include "Panel.hpp"

namespace gui {
    class CheckBox : public UINode {
    protected:
        glm::vec4 checkColor {1.0f, 1.0f, 1.0f, 0.4f};
        boolsupplier supplier = nullptr;
        boolconsumer consumer = nullptr;
        bool checked = false;
    public:
        explicit CheckBox(GUI& gui, bool checked = false);

        virtual void draw(const DrawContext& pctx, const Assets& assets)
            override;

        virtual void mouseRelease(int x, int y) override;

        virtual void setSupplier(boolsupplier supplier);
        virtual void setConsumer(boolconsumer consumer);

        virtual CheckBox* setChecked(bool flag);

        virtual bool isChecked() const {
            if (supplier) return supplier();
            return checked;
        }
    };

    class FullCheckBox : public Panel {
    protected:
        std::shared_ptr<CheckBox> checkbox;
        std::shared_ptr<Label> label;
    public:
        explicit FullCheckBox(
            GUI& gui,
            const std::wstring& text,
            glm::vec2 size,
            bool checked = false
        );

        virtual void setSupplier(boolsupplier supplier) {
            checkbox->setSupplier(std::move(supplier));
        }

        virtual void setConsumer(boolconsumer consumer) {
            checkbox->setConsumer(std::move(consumer));
        }

        virtual void setChecked(bool flag) {
            checkbox->setChecked(flag);
        }

        virtual bool isChecked() const {
            return checkbox->isChecked();
        }

        virtual void setTooltip(const std::wstring& text) override {
            Panel::setTooltip(text);
            checkbox->setTooltip(text);
        }

        virtual void setSize(const glm::vec2& new_size) override {
            Panel::setSize(new_size);
            checkbox->setSize(glm::vec2(size.y, size.y));
        }
    };
}
