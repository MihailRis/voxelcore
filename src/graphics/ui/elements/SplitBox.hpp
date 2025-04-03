#pragma once

#include "BasePanel.hpp"

namespace gui {
    class SplitBox : public BasePanel {
    public:
        SplitBox(GUI& gui, const glm::vec2& size, float splitPos, Orientation orientation);
        
        virtual void mouseMove(int x, int y) override;
        virtual void refresh() override;
        virtual void fullRefresh() override;
        virtual void doubleClick(int x, int y) override;
    private:
        float splitPos;
    };
}
