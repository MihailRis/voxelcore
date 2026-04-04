#pragma once

#include "Container.hpp"

namespace gui {
    class BasePanel : public Container {
    public:
        virtual ~BasePanel() = default;

        virtual void setOrientation(Orientation orientation) {
            this->orientation = orientation;
            refresh();
        }

        Orientation getOrientation() const {
            return orientation;
        }

        virtual void setPadding(glm::vec4 padding) {
            this->padding = padding;
            refresh();
        }

        glm::vec4 getPadding() const {
            return padding;
        }
    protected:
        BasePanel(
            GUI& gui,
            glm::vec2 size,
            glm::vec4 padding = glm::vec4(0.0f),
            float interval = 2.0f,
            Orientation orientation = Orientation::VERTICAL
        )
            : Container(gui, std::move(size)),
            padding(std::move(padding)),
            interval(interval) {
        }

        Orientation orientation = Orientation::VERTICAL;
        glm::vec4 padding;
        float interval = 2.0f;
    };
}
