#include "Layout.hpp"

void Layout::apply_styles_to_layout(Node& node) {
    LayoutBox& layout = node.get_layout();

    if (auto* element = std::get_if<Element>(&node.node_type)) {
        const auto& style = element->style;

        // Размеры
        layout.content_size.x =
            get_dimension_from_style(style, "width", layout.content_size.x);
        layout.content_size.y =
            get_dimension_from_style(style, "height", layout.content_size.y);

        // Минимальные размеры
        layout.min_size.x = get_dimension_from_style(style, "min-width", 0);
        layout.min_size.y = get_dimension_from_style(style, "min-height", 0);

        // Максимальные размеры
        layout.max_size.x =
            get_dimension_from_style(style, "max-width", FLT_MAX);
        layout.max_size.y =
            get_dimension_from_style(style, "max-height", FLT_MAX);

        // Padding
        layout.padding.x = get_dimension_from_style(style, "padding-left", 0) +
                           get_dimension_from_style(style, "padding-right", 0);
        layout.padding.y = get_dimension_from_style(style, "padding-top", 0) +
                           get_dimension_from_style(style, "padding-bottom", 0);

        // Margin
        layout.margin.x = get_dimension_from_style(style, "margin-left", 0) +
                          get_dimension_from_style(style, "margin-right", 0);
        layout.margin.y = get_dimension_from_style(style, "margin-top", 0) +
                          get_dimension_from_style(style, "margin-bottom", 0);

        // Border
        layout.border.x =
            get_dimension_from_style(style, "border-left-width", 0) +
            get_dimension_from_style(style, "border-right-width", 0);
        layout.border.y =
            get_dimension_from_style(style, "border-top-width", 0) +
            get_dimension_from_style(style, "border-bottom-width", 0);
    }
}

float Layout::get_dimension_from_style(
    const style::ComputedStyle& style,
    const std::string& property,
    float default_value
) {
    if (auto value = style.get(property)) {
        return value->asFloat(0.0);
    }
    return default_value;
}

void Layout::compute_layout(Node& node, const LayoutContext& context) {
    // Если layout не нуждается в обновлении, пропускаем
    if (!node.is_layout_dirty()) {
        return;
    }

    // Вычисляем layout для текущего узла
    std::visit(
        [&node, &context](auto& node_type) {
            using T = std::decay_t<decltype(node_type)>;
            if constexpr (std::is_same_v<T, Element>) {
                compute_element_layout(node, node_type, context);
            } else if constexpr (std::is_same_v<T, Text>) {
                compute_text_layout(node, node_type, context);
            }
        },
        node.node_type
    );

    // Помечаем узел как "чистый"
    node.mark_layout_clean();

    // Рекурсивно вычисляем layout для детей
    LayoutContext child_context = context;
    child_context.parent_layout = &node.get_layout();
    child_context.parent_position = node.get_layout().position;

    for (auto& child : node.children) {
        compute_layout(child, child_context);
    }
}

void Layout::compute_element_layout(
    Node& node, Element& element, const LayoutContext& context
) {
    apply_styles_to_layout(node);
    compute_block_layout(node, element, context);
}

void Layout::compute_text_layout(
    Node& node, Text& text, const LayoutContext& context
) {
    LayoutBox& layout = node.get_layout();

    // Для текста вычисляем размер на основе содержимого
    // Пока используем приблизительные значения
    float text_width = text.content.length() * 8.0f;  // ~8px на символ
    float text_height = 16.0f;                        // высота строки

    layout.content_size = {text_width, text_height};
    layout.position = context.parent_position;

    // Если есть родитель, позиционируем относительно него
    if (context.parent_layout) {
        // Простое позиционирование - можно улучшить
        layout.position.x = context.parent_position.x;
        layout.position.y = context.parent_position.y;
    }
}

void Layout::compute_block_layout(
    Node& node, Element& element, const LayoutContext& context
) {
    LayoutBox& layout = node.get_layout();

    // Устанавливаем позицию
    layout.position = context.parent_position;

    // Если есть родитель, учитываем его padding
    if (context.parent_layout) {
        layout.position.x += context.parent_layout->padding.x;
        layout.position.y += context.parent_layout->padding.y;
    }

    // Вычисляем размеры детей рекурсивно
    float max_child_width = 0.0f;
    float total_child_height = 0.0f;

    // Сначала вычисляем layout для всех детей
    LayoutContext child_context = context;
    child_context.parent_position = {
        layout.position.x, layout.position.y + total_child_height
    };

    for (auto& child : node.children) {
        compute_layout(child, child_context);
        const LayoutBox& child_layout = child.get_layout();

        max_child_width =
            glm::max(max_child_width, child_layout.get_outer_size().x);
        total_child_height += child_layout.get_outer_size().y;
    }

    // Устанавливаем размеры элемента
    float specified_width = layout.content_size.x;
    float specified_height = layout.content_size.y;

    // Если ширина не задана, используем максимальную ширину детей или доступное
    // пространство
    if (specified_width <= 0) {
        if (max_child_width > 0) {
            layout.content_size.x = max_child_width;
        } else {
            layout.content_size.x = context.available_space.x > 0
                                        ? context.available_space.x
                                        : 100.0f;  // дефолт
        }
    }

    // Если высота не задана, используем высоту детей
    if (specified_height <= 0) {
        layout.content_size.y =
            total_child_height > 0 ? total_child_height : 20.0f;  // дефолт
    }

    // Позиционируем детей вертикально (block layout)
    float current_y = layout.position.y + layout.padding.y;
    for (auto& child : node.children) {
        LayoutBox& child_layout = child.get_layout();
        child_layout.position.x = layout.position.x + layout.padding.x;
        child_layout.position.y = current_y;
        current_y += child_layout.get_outer_size().y;
    }
}