#include "Layout.hpp"
#include "assets/Assets.hpp"
#include "constants.hpp"

#include <variant>

// Основной метод вычисления layout
void Layout::compute_layout(Node& node, LayoutContext& context, const Assets& assets) {
    if (!node.is_layout_dirty()) return;

    std::visit(
        [&](auto& inner) {
            using T = std::decay_t<decltype(inner)>;
            if constexpr (std::is_same_v<T, Text>) {
                compute_text_layout(node, inner, context, assets);
            } else if constexpr (std::is_same_v<T, Element>) {
                compute_element_layout(node, inner, context, assets);
            }
        },
        node.node_type
    );

    node.mark_layout_clean();
}

// --- Текстовый layout ---
void Layout::compute_text_layout(
    Node& node, Text& text, LayoutContext& context, const Assets& assets
) {
    auto font = assets.get<Font>(FONT_DEFAULT);
    float text_width = font->calcWidth(util::str2wstr_utf8(text.content), 0,text.content.size());
    float text_height = node.layout.font_size;

    node.layout.width = text_width;
    node.layout.height = text_height;

    if (auto p = node.parent.lock()) {  // проверяем, что родитель существует
        node.layout.x = p->layout.x;
        node.layout.y = p->layout.y;
    }
}

// --- Создание контекста для детей ---
LayoutContext Layout::create_child_context(
    Node& node, float content_width, float max_height
) {
    LayoutContext ctx;
    ctx.parent_layout = &node.layout;
    ctx.available_space = {
        static_cast<unsigned>(content_width), static_cast<unsigned>(max_height)
    };
    return ctx;
}

// --- Основной layout для элемента ---
void Layout::compute_element_layout(
    Node& node, Element& element, LayoutContext& context, const Assets& assets
) {
    // 1) Позиционируем элемент

    float max_width = static_cast<float>(context.available_space.x);
    float max_height = static_cast<float>(context.available_space.y);

    // 2) Вычисляем размеры элемента
    node.layout.width =
        element.style.get("width", max_width).asFloat(max_width);
    node.layout.height = element.style.get("height", max_height)
                             .asFloat(max_height);  // временно

    bool vertical =
        element.style.get("direction", "column").asString("row") ==
        "row";

    // 3) Раскладываем детей по flex

    float main_cursor = 0.f;  // основная ось

    LayoutContext child_ctx;
    
    if (vertical) child_ctx = create_child_context(node, node.layout.width, node.layout.height / node.children.size());
    else child_ctx = create_child_context(node, node.layout.width / node.children.size(), node.layout.height );

    for (auto& child : node.children) {
        if (vertical) {
            child->layout.x = node.layout.x;
            child->layout.y = node.layout.y + main_cursor;
            compute_layout(*child, child_ctx, assets);
            main_cursor += child->layout.height;
        } else {
            child->layout.x = node.layout.x + main_cursor;
            child->layout.y = node.layout.y;
            compute_layout(*child, child_ctx, assets);
            main_cursor += child->layout.width;
        }
    }
}
