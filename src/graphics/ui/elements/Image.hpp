#pragma once

#include "UINode.hpp"
#include "maths/UVRegion.hpp"

namespace util {
    struct TextureRegion;
}

namespace gui {
    class Image final : public UINode {
    protected:
        std::string texture;
        std::string fallback;
        UVRegion region {};
        bool autoresize = false;

        util::TextureRegion refreshTexture(const Assets& assets);
    public:
        Image(GUI& gui, std::string texture, glm::vec2 size=glm::vec2(32,32));

        void draw(const DrawContext& pctx, const Assets& assets) override;

        void setAutoResize(bool flag);
        bool isAutoResize() const;
        const std::string& getTexture() const;
        const std::string& getFallback() const;
        void setTexture(const std::string& name);
        void setFallback(const std::string& name);
        void setRegion(const UVRegion& region);
        const UVRegion& getRegion() const;
    };
}
