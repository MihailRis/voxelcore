#pragma once

#include "Button.hpp"

namespace gui {
    class Label;

    class SelectBox : public Button {
    public:
        struct Option {
            std::string value;
            std::wstring text;
        };
        enum class Mode {
            SELECT,
            BUTTON,
        };
    private:
        std::vector<Option> options;
        Option selected {};
        StringCallbacksSet changeCallbacks;
        Mode mode;
    public:
        SelectBox(
            GUI& gui,
            std::vector<Option>&& elements,
            Option selected,
            Mode mode,
            int contentWidth,
            const glm::vec4& padding
        );

        void listenChange(OnStringChange&& callback);

        void setSelected(const Option& selected);

        const Option& getSelected() const;

        const std::vector<Option>& getOptions() const;

        void setOptions(std::vector<Option>&& options);

        Mode getMode() const;

        void setMode(Mode mode);

        void drawBackground(const DrawContext& pctx, const Assets&) override;


        std::shared_ptr<Label> getLabel() const;
    };
}
