#pragma once

#include "UINode.hpp"

namespace gui {
    class TrackBar : public UINode {
    protected:
        glm::vec4 trackColor {1.0f, 1.0f, 1.0f, 0.4f};
        doublesupplier supplier = nullptr;
        doubleconsumer consumer = nullptr;
        doubleconsumer subconsumer = nullptr;
        double min;
        double max;
        double value;
        double step;
        int trackWidth;
        bool changeOnRelease = false;
    public:
        TrackBar(
            GUI& gui,
            double min,
            double max,
            double value,
            double step = 1.0,
            int trackWidth = 12
        );
        virtual void draw(const DrawContext& pctx, const Assets& assets) override;

        virtual void setSupplier(doublesupplier);
        virtual void setConsumer(doubleconsumer);
        virtual void setSubConsumer(doubleconsumer);

        virtual void mouseMove(int x, int y) override;
        virtual void mouseRelease(int x, int y) override;

        virtual double getValue() const;
        virtual double getMin() const;
        virtual double getMax() const;
        virtual double getStep() const;
        virtual int getTrackWidth() const;
        virtual glm::vec4 getTrackColor() const;
        virtual bool isChangeOnRelease() const;

        virtual void setValue(double);
        virtual void setMin(double);
        virtual void setMax(double);
        virtual void setStep(double);
        virtual void setTrackWidth(int);
        virtual void setTrackColor(glm::vec4);
        virtual void setChangeOnRelease(bool);
    };
}
