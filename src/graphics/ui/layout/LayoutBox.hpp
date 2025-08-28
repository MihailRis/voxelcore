#pragma once
#include <glm/glm.hpp>

#include "graphics/ui/style/Stylesheet.h"

struct LayoutBox {
    float x = 0;
    float y = 0;
    float width = 0;
    float height = 0;
    float font_size = 16;
};

struct LayoutContext {
    const LayoutBox* parent_layout = nullptr;
    // const LayoutBox* current_layout = nullptr;
    glm::uvec2 available_space;
};