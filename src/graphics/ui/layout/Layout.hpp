#pragma once
#include "../elements/Node.hpp"
#include "LayoutBox.hpp"

class Layout {
public:
    glm::uvec2 size = glm::uvec2(0, 0);

    // Основной метод вычисления layout
    static void compute_layout(Node& node, LayoutContext& context, const Assets& assets);

    // Методы для разных типов layout
    static void compute_text_layout(
        Node& node, Text& text, LayoutContext& context, const Assets& assets
    );
    static void compute_element_layout(
        Node& node, Element& element, LayoutContext& context, const Assets& assets
    );

private:
    static void set_element_position(Node& node, LayoutContext& context);
    static LayoutContext create_child_context(
        Node& node, float content_width, float max_height
    );
};