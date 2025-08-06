#pragma once
#include "LayoutBox.hpp"
#include "../elements/Node.hpp"

class Layout {
public:
    struct LayoutContext {
        glm::vec2 available_space;
        glm::vec2 parent_position;
        LayoutBox* parent_layout = nullptr;
    };

    // Основной метод вычисления layout
    static void compute_layout(Node& node, const LayoutContext& context);

    // Методы для разных типов layout
    static void compute_text_layout(Node& node, Text& text, const LayoutContext& context);
    static void compute_element_layout(Node& node, Element& element, const LayoutContext& context);
    static void compute_block_layout(Node& node, Element& element, const LayoutContext& context);

    // Утилиты
    static LayoutBox& get_layout_box(Node& node);
    static void apply_styles_to_layout(Node& node);
    static glm::vec2 calculate_preferred_size(
        Node& node, const LayoutContext& context
    );
private:
    // Вспомогательные методы
    static void calculate_margins(Node& node, const LayoutContext& context);
    static void calculate_padding(Node& node);
    static void calculate_borders(Node& node);
    static void position_children(Node& node);

    // Работа со стилями
    static float get_dimension_from_style(
        const style::ComputedStyle& style,
        const std::string& property,
        float default_value
    );
    static glm::vec2 get_dimensions_from_style(
        const style::ComputedStyle& style,
        const std::string& width_prop,
        const std::string& height_prop,
        const glm::vec2& default_size
    );
};