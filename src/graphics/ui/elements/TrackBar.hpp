#pragma once

#include "UINode.hpp"

namespace gui {
    class TrackBar final : public UINode {
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
        void draw(const DrawContext& pctx, const Assets& assets) override;

        void setSupplier(doublesupplier);
        void setConsumer(doubleconsumer);
        void setSubConsumer(doubleconsumer);

        void mouseMove(int x, int y) override;
        void mouseRelease(int x, int y) override;

        double getValue() const;
        double getMin() const;
        double getMax() const;
        double getStep() const;
        int getTrackWidth() const;
        const glm::vec4& getTrackColor() const;
        bool isChangeOnRelease() const;

        void setValue(double);
        void setMin(double);
        void setMax(double);
        void setStep(double);
        void setTrackWidth(int);
        void setTrackColor(glm::vec4);
        void setChangeOnRelease(bool);
    };
}
