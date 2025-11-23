#include "Canvas.hpp"

#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Texture.hpp"

gui::Canvas::Canvas(GUI& gui, ImageFormat format, glm::uvec2 size)
    : UINode(gui, size) {
    auto data = std::make_shared<ImageData>(format, size.x, size.y);
    texture = Texture::from(data.get());
    this->data = std::move(data);
}

void gui::Canvas::draw(const DrawContext& pctx, const Assets& assets) {
    auto pos = calcPos();
    auto col = calcColor();

    auto batch = pctx.getBatch2D();
    batch->texture(texture.get());
    batch->rect(pos.x, pos.y, size.x, size.y, 0, 0, 0, {}, false, false, col);
}

void gui::Canvas::setSize(const glm::vec2& size) {
    UINode::setSize(size);
    data->extend(size.x, size.y);
    texture->reload(*data);
}
