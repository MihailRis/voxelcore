#pragma once
#include <glm/glm.hpp>
#include "graphics/ui/style/Stylesheet.h"

struct LayoutBox {
    glm::vec2 content_size = {0, 0};     // Размер контента
    glm::vec2 position = {0, 0};         // Абсолютная позиция
    glm::vec2 margin = {0, 0};           // Внешние отступы
    glm::vec2 padding = {0, 0};          // Внутренние отступы
    glm::vec2 border = {0, 0};           // Границы
    glm::vec2 full_size = {0, 0};        // Полный размер (content + padding + border)
    
    // Минимальные и максимальные размеры
    glm::vec2 min_size = {0, 0};
    glm::vec2 max_size = {FLT_MAX, FLT_MAX};
    
    // Preferred size (для flexbox и т.д.)
    std::optional<glm::vec2> preferred_size;
    
    // Флаги
    bool dirty = true;                   // Нужно пересчитать layout
    bool visible = true;
    
    // Методы для работы с размерами
    glm::vec2 get_content_box() const {
        return content_size;
    }
    
    glm::vec2 get_padding_box() const {
        return {content_size.x + padding.x * 2, content_size.y + padding.y * 2};
    }
    
    glm::vec2 get_border_box() const {
        return {get_padding_box().x + border.x * 2, get_padding_box().y + border.y * 2};
    }
    
    glm::vec2 get_margin_box() const {
        return {get_border_box().x + margin.x * 2, get_border_box().y + margin.y * 2};
    }
    
    // Утилиты
    glm::vec2 get_inner_size() const {
        return content_size;
    }
    
    glm::vec2 get_outer_size() const {
        return get_margin_box();
    }
    
    glm::vec4 get_content_rect() const {
        return {position.x, position.y, content_size.x, content_size.y};
    }
    
    glm::vec4 get_padding_rect() const {
        return {position.x - padding.x, position.y - padding.y, 
                get_padding_box().x, get_padding_box().y};
    }
};