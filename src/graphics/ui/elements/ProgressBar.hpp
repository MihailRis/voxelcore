#pragma once

#include <string>

#include "UINode.hpp"
#include "commons.hpp"

namespace gui {
    class ProgressBar : public UINode {
    protected:
        glm::vec4 barColor {0.2f, 0.6f, 1.0f, 0.8f};
        glm::vec4 bgColor {0.0f, 0.0f, 0.0f, 0.4f};
        glm::vec4 textColor {1.0f, 1.0f, 1.0f, 1.0f};
        doublesupplier supplier = nullptr;
        double min;
        double max;
        double value;
        double displayValue;
        int barThickness;
        float smoothSpeed = 120.0f;
        Orientation orientation = Orientation::HORIZONTAL;
        bool textVisible = false;
        bool smoothTransition = false;
        std::string textFormat;
    public:
        /** min, max, value, displayValue — числовой диапазон и отображаемое значение
         *  textFormat — формат sprintf. Аргументы:
         *    %1$ — displayValue (сырое значение в диапазоне min–max)
         *    %2$ — прогресс в процентах 0..100
         *  Пример: "%.0f%%" покажет "45%", "%2$.0f%%" покажет "62%" при min=10 max=50
         */
        ProgressBar(
            GUI& gui,
            double min = 0.0,
            double max = 100.0,
            double value = 0.0,
            int barThickness = -1
        );
        virtual void act(float delta) override;
        virtual void draw(const DrawContext& pctx, const Assets& assets) override;

        virtual void setSupplier(doublesupplier);
        virtual double getValue() const;
        virtual double getMin() const;
        virtual double getMax() const;
        virtual double getProgress() const;
        virtual glm::vec4 getBarColor() const;
        virtual glm::vec4 getBgColor() const;
        virtual glm::vec4 getTextColor() const;
        virtual Orientation getOrientation() const;
        virtual const std::string& getTextFormat() const;
        virtual bool isSmooth() const;

        virtual void setValue(double);
        virtual void setMin(double);
        virtual void setMax(double);
        virtual void setBarThickness(int);
        virtual void setBarColor(glm::vec4);
        virtual void setBgColor(glm::vec4);
        virtual void setTextColor(glm::vec4);
        virtual void setTextFormat(const std::string& fmt);
        virtual void setSmooth(bool flag);
        virtual void setOrientation(Orientation);
    };
}
