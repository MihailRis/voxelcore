#pragma once
#include "../elements/Node.hpp"
#include "LayoutBox.hpp"

enum class FlexDirection { Column, Row };

struct BoxSides {
    float left = 0;
    float top = 0;
    float right = 0;
    float bottom = 0;

    // Конструктор по всем четырем сторонам
    BoxSides(float l, float t, float r, float b)
        : left(l), top(t), right(r), bottom(b) {
    }

    // Конструктор с одинаковым значением для всех сторон
    explicit BoxSides(float all)
        : left(all), top(all), right(all), bottom(all) {
    }

    // Конструктор по осям (горизонталь, вертикаль)
    BoxSides(float horizontal, float vertical)
        : left(horizontal / 2),
          right(horizontal / 2),
          top(vertical / 2),
          bottom(vertical / 2) {
    }

    float horizontal() const {
        return left + right;
    }
    float vertical() const {
        return top + bottom;
    }

    BoxSides merge(const BoxSides& other) const {
        return BoxSides(
            left + other.left,
            top + other.top,
            right + other.right,
            bottom + other.bottom
        );
    }
};

class Layout {
public:
    // Главный вызов
    static void compute_layout(
        Node& root, LayoutContext& ctx, const Assets& assets
    );
private:
    static void measure(
        Node& node, const LayoutContext& ctx, const Assets& assets
    );
    static void measure_text(
        Node& node, Text& text, const LayoutContext& ctx, const Assets& assets
    );
    static void measure_element(
        Node& node,
        Element& element,
        const LayoutContext& ctx,
        const Assets& assets
    );
    static void position_node(Node& node, const glm::vec2& offset);
    static void position_text(Node& node, Text&, const glm::vec2& offset);
    static void position_element(
        Node& node, Element& element, const glm::vec2& offset
    );

    static FlexDirection get_direction(Element& element);
    static BoxSides get_padding(const style::ComputedStyle& style);
    static BoxSides get_margin(const style::ComputedStyle& style);
    static BoxSides get_border(const style::ComputedStyle& style);

    static BoxSides get_child_margin(Node& child);
    static float get_child_main_size(const LayoutBox& cl, const BoxSides& cm, FlexDirection dir);
    static float get_child_cross_size(const LayoutBox& cl, const BoxSides& cm, FlexDirection dir);
    static float initial_cursor(float container_main, float total_main, const std::string& align);
    static float cross_offset(float container_cross, float child_cross, const std::string& align);
};