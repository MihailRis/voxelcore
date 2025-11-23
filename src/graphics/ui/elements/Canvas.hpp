#pragma once

#include "UINode.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"

class Texture;

namespace gui {
    class Canvas final : public UINode {
    public:
        explicit Canvas(GUI& gui, ImageFormat format, glm::uvec2 size);

        ~Canvas() override = default;

        void draw(const DrawContext& pctx, const Assets& assets) override;

        void setSize(const glm::vec2& size) override;

        [[nodiscard]] auto getTexture() const {
            return texture;
        }

        [[nodiscard]] auto getData() const {
            return data;
        }
    private:
        std::shared_ptr<::Texture> texture;
        std::shared_ptr<ImageData> data;
    };
}
