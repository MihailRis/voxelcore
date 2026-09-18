#pragma once

#include <string>

#include "UINode.hpp"
#include "commons.hpp"

namespace gui {
    class ProgressBar final : public UINode {
    protected:
        glm::vec4 bgColor {0.0f, 0.0f, 0.0f, 0.4f};
        glm::vec4 textColor {1.0f, 1.0f, 1.0f, 1.0f};
        std::wstring text;
        double min;
        double max;
        double value;
        double displayValue;
        float smoothSpeed = 100.0f;
        Orientation orientation = Orientation::HORIZONTAL;
        bool smoothTransition = false;
        std::string fontName = "normal";
    public:
        ProgressBar(
            GUI& gui,
            double min = 0.0,
            double max = 100.0,
            double value = 0.0
        );
        void act(float delta) override;
        void draw(const DrawContext& pctx, const Assets& assets) override;

        double getValue() const;
        double getMin() const;
        double getMax() const;
        double getProgress() const;
        double getDisplayValue() const;
        glm::vec4 getBgColor() const;
        glm::vec4 getTextColor() const;
        Orientation getOrientation() const;
        const std::wstring& getText() const;
        const std::string& getFontName() const;
        bool isSmooth() const;
        float getSmoothSpeed() const;

        void setValue(double);
        void setMin(double);
        void setMax(double);
        void setBgColor(glm::vec4);
        void setTextColor(glm::vec4);
        void setText(std::wstring text);
        void setFontName(std::string name);
        void setSmooth(bool flag);
        void setSmoothSpeed(float speed);
        void setOrientation(Orientation);
    };
}
