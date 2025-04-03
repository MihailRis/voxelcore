#pragma once

#include "UINode.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"

class Texture;

namespace gui {
    class Canvas final : public UINode {
    public:
        explicit Canvas(GUI& gui, ImageFormat inFormat, glm::uvec2 inSize);

        ~Canvas() override = default;

        void draw(const DrawContext& pctx, const Assets& assets) override;

        [[nodiscard]] auto texture() const {
            return mTexture;
        }

        [[nodiscard]] auto data() const {
            return mData;
        }
    private:
        std::shared_ptr<::Texture> mTexture;
        std::shared_ptr<ImageData> mData;
    };
}
