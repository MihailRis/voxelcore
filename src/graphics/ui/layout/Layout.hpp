#pragma once
#include "../elements/Node.hpp"
#include "LayoutBox.hpp"

class Layout {
public:
    glm::uvec2 size = glm::uvec2(0, 0);

    // Основной метод вычисления layout
    static void compute_layout(Node& node, LayoutContext& context);

    // Методы для разных типов layout
    static void compute_text_layout(
        Node& node, Text& text, LayoutContext& context
    );
    static void compute_element_layout(
        Node& node, Element& element, LayoutContext& context
    );
private:
    static float compute_element_content_width(
        const Element& element, float max_width
    );
    static float compute_element_content_height(
        const Element& element, float children_height
    );
    static void set_element_position(Node& node, LayoutContext& context);
    static LayoutContext create_child_context(
        Node& node, float content_width, float max_height
    );
    static float layout_children(Node& node, LayoutContext& child_ctx);
    static float layout_children_flex(Node& node, LayoutContext& ctx);
};