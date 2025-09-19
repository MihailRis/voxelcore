#pragma once
#include <glm/glm.hpp>

#include "graphics/ui/style/Stylesheet.h"

struct LayoutBox {
    glm::vec2 content_size = glm::vec2(0, 0);
    glm::vec2 size = glm::vec2(0, 0);
    glm::vec2 position = glm::vec2(0, 0);

    glm::vec2 padding = {0, 0};  // left+right, top+bottom
    glm::vec2 margin = {0, 0};   // left+right, top+bottom
    glm::vec2 border = {0, 0};   // left+right, top+bottom
};

struct LayoutContext {
    const LayoutBox* parent_layout = nullptr;
    glm::vec2 available_space;
};