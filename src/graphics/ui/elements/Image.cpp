#include "Image.hpp"

#include <utility>

#include "assets/Assets.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Texture.hpp"
#include "maths/UVRegion.hpp"

using namespace gui;

Image::Image(GUI& gui, std::string texture, glm::vec2 size)
    : UINode(gui, size), texture(std::move(texture)) {
    setInteractive(false);
}

void Image::draw(const DrawContext& pctx, const Assets& assets) {
    glm::vec2 pos = calcPos();
    auto batch = pctx.getBatch2D();

    Texture* texture = nullptr;
    auto separator = this->texture.find(':');
    if (separator == std::string::npos) {
        texture = assets.get<Texture>(this->texture);
        batch->texture(texture);
        if (texture && autoresize) {
            setSize(glm::vec2(texture->getWidth(), texture->getHeight()));
        }
    } else {
        auto atlasName = this->texture.substr(0, separator);
        if (auto atlas = assets.get<Atlas>(atlasName)) {
            if (auto region =
                    atlas->getIf(this->texture.substr(separator + 1))) {
                texture = atlas->getTexture();
                batch->texture(atlas->getTexture());
                batch->setRegion(*region);
                if (autoresize) {
                    setSize(glm::vec2(
                        texture->getWidth() * region->getWidth(),
                        texture->getHeight() * region->getHeight()
                    ));
                }
            } else {
                batch->texture(nullptr);
            }
        }
    }
    batch->rect(
        pos.x,
        pos.y,
        size.x,
        size.y,
        0,
        0,
        0,
        region,
        false,
        true,
        calcColor()
    );
}

void Image::setAutoResize(bool flag) {
    autoresize = flag;
}
bool Image::isAutoResize() const {
    return autoresize;
}

const std::string& Image::getTexture() const {
    return texture;
}

void Image::setTexture(const std::string& name) {
    texture = name;
}

void Image::setRegion(const UVRegion& region) {
    this->region = region;
}

const UVRegion& Image::getRegion() const {
    return region;
}
