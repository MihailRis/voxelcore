#pragma once

#include "Container.hpp"

class Framebuffer;
class UiDocument;
class ImageData;

namespace gui {
    class Frame final : public Container {
    public:
        Frame(GUI& gui, std::string id, std::string outputTexture);
        ~Frame();

        void draw(const DrawContext& pctx, const Assets& assets) override;

        void updateOutput(Assets& assets);

        const std::string& getOutputTexture() const;

        const std::string& getFrameId() const;

        std::unique_ptr<ImageData> takeScreenshot() const;
    private:
        std::string frameId;
        std::unique_ptr<Framebuffer> fbo;
        std::string outputTexture;
    };
}
