#include "Layout.hpp"

#include <cmath>
#include <variant>

#include "assets/Assets.hpp"
#include "constants.hpp"

void Layout::compute_layout(
    Node& root, LayoutContext& ctx, const Assets& assets
) {
    measure(root, ctx, assets);

    position_node(root, glm::vec2(0, 0));
}

void Layout::measure(
    Node& node, const LayoutContext& ctx, const Assets& assets
) {
    std::visit(
        [&](auto& inner) {
            using T = std::decay_t<decltype(inner)>;
            if constexpr (std::is_same_v<T, Text>) {
                measure_text(node, inner, ctx, assets);
            } else if constexpr (std::is_same_v<T, Element>) {
                measure_element(node, inner, ctx, assets);
            }
        },
        node.node_type
    );
}

// --- Текст ---
void Layout::measure_text(
    Node& node, Text& text, const LayoutContext& ctx, const Assets& assets
) {
    auto font = assets.get<Font>(FONT_DEFAULT);
    float w = font->calcWidth(
        util::str2wstr_utf8(text.content), 0, text.content.size()
    );
    float h = 16;

    glm::vec2 size = {w, h};
    node.layout.content_size = size;
    node.layout.size = size;
}

// --- Элемент ---
void Layout::measure_element(
    Node& node, Element& element, const LayoutContext& ctx, const Assets& assets
) {
    using std::max;
    using std::numeric_limits;

    FlexDirection dir = get_direction(element);

    BoxSides padding = get_padding(element.style);
    BoxSides border = get_border(element.style);
    BoxSides margin = get_margin(
        element.style
    );  // margin контейнера (НЕ добавлять в node.layout.size!)

    // 1) Доступное пространство внутри border + padding (content box)
    glm::vec2 content_available = ctx.available_space;
    content_available.x =
        max(0.0f,
            content_available.x - (border.horizontal() + padding.horizontal()));
    content_available.y =
        max(0.0f,
            content_available.y - (border.vertical() + padding.vertical()));

    // Первый проход: измеряем детей так, чтобы получить их intrinsic cross-axis
    // размеры.
    float total_fixed = 0.0f;
    size_t flexible_count = 0;

    std::vector<BoxSides> child_margins;
    child_margins.reserve(node.children.size());

    for (auto& child : node.children) {
        BoxSides cm(0.0f);
        Element* child_el = nullptr;
        if (auto* nt = std::get_if<Element>(&child->node_type)) {
            child_el = nt;
            cm = get_margin(child_el->style);
        }
        child_margins.push_back(cm);

        LayoutContext child_ctx;
        child_ctx.parent_layout = &node.layout;

        // Передаём на cross-оси реальную доступную ширину/высоту,
        // а на main-оси — "неограниченно", чтобы получить intrinsic size.
        glm::vec2 child_avail = content_available;
        if (dir == FlexDirection::Column) {
            child_avail.x = max(
                0.0f, child_avail.x - cm.horizontal()
            );  // cross constrained by container content width minus child
                // margins
            child_avail.y =
                numeric_limits<float>::infinity();  // allow child to measure
                                                    // its intrinsic height
        } else {                                    // Row
            child_avail.y =
                max(0.0f, child_avail.y - cm.vertical());  // cross constrained
            child_avail.x =
                numeric_limits<float>::infinity();  // allow intrinsic width
        }
        child_ctx.available_space = child_avail;

        measure(*child, child_ctx, assets);

        auto& cl = child->get_layout();
        if (dir == FlexDirection::Column) {
            if (child_el && child_el->style.has("height"))
                total_fixed += cl.size.y + cm.vertical();
            else
                flexible_count++;
        } else {
            if (child_el && child_el->style.has("width"))
                total_fixed += cl.size.x + cm.horizontal();
            else
                flexible_count++;
        }
    }

    // 2) Вычисляем stretch_size для гибких детей
    float remaining_main =
        (dir == FlexDirection::Column ? content_available.y
                                      : content_available.x) -
        total_fixed;
    remaining_main = max(0.0f, remaining_main);
    float stretch_size =
        (flexible_count > 0) ? (remaining_main / float(flexible_count)) : 0.0f;

    // 3) Второй проход: перемеряем гибких детей, передав им конкретный
    // main-size = stretch_size
    for (size_t i = 0; i < node.children.size(); ++i) {
        auto& child = node.children[i];
        auto& cm = child_margins[i];
        Element* child_el = nullptr;
        if (auto* nt = std::get_if<Element>(&child->node_type)) child_el = nt;

        bool is_fixed_main = (dir == FlexDirection::Column)
                                 ? (child_el && child_el->style.has("height"))
                                 : (child_el && child_el->style.has("width"));

        if (!is_fixed_main) {
            LayoutContext child_ctx;
            child_ctx.parent_layout = &node.layout;

            glm::vec2 child_avail = content_available;
            if (dir == FlexDirection::Column) {
                // main available = stretch_size минус margin вертикальные (не
                // может быть отрицательным)
                child_avail.y = max(0.0f, stretch_size - cm.vertical());
                child_avail.x = max(0.0f, child_avail.x - cm.horizontal());
            } else {
                child_avail.x = max(0.0f, stretch_size - cm.horizontal());
                child_avail.y = max(0.0f, child_avail.y - cm.vertical());
            }
            child_ctx.available_space = child_avail;

            measure(*child, child_ctx, assets);

            // Убедимся, что главный размер ребёнка = stretch_size
            // (border/padding внутри child.get_layout().size уже учтены)
            auto& cl = child->get_layout();
            if (dir == FlexDirection::Column)
                cl.size.y = stretch_size;
            else
                cl.size.x = stretch_size;
        } else {
            // у фиксированных детей можно (опционально) убедиться, что их
            // размер не превышает content_available
            auto& cl = child->get_layout();
            if (dir == FlexDirection::Column)
                cl.size.y = std::min(cl.size.y, content_available.y);
            else
                cl.size.x = std::min(cl.size.x, content_available.x);
        }
    }

    // 4) Вычисляем content_size контейнера (children sizes + margins!)
    glm::vec2 content_size(0.0f);
    for (size_t i = 0; i < node.children.size(); ++i) {
        auto& child = node.children[i];
        auto& cl = child->get_layout();
        const BoxSides& cm = child_margins[i];

        if (dir == FlexDirection::Column) {
            content_size.x =
                max(content_size.x,
                    cl.size.x + cm.horizontal());  // add margins
            content_size.y += cl.size.y + cm.vertical();
        } else {
            content_size.x += cl.size.x + cm.horizontal();
            content_size.y = max(content_size.y, cl.size.y + cm.vertical());
        }
    }

    glm::vec2 final_content = content_size;

    if (element.style.has("width"))
        final_content.x =
            element.style.get("width", 0).asFloat(final_content.x);

    if (element.style.has("height"))
        final_content.y =
            element.style.get("height", 0).asFloat(final_content.y);

    // 6) Если размеры не заданы — растягиваем
    if (!element.style.has("width")) {
        if (dir == FlexDirection::Row) {
            // main-ось = X → растягиваем по X
            final_content.x = std::max(content_size.x, content_available.x);
        } else {
            // cross-ось = X → растягиваем на всё
            final_content.x = content_available.x;
        }
    }

    if (!element.style.has("height")) {
        if (dir == FlexDirection::Column) {
            // main-ось = Y → растягиваем по Y
            final_content.y = std::max(content_size.y, content_available.y);
        } else {
            // cross-ось = Y → растягиваем на всё
            final_content.y = content_available.y;
        }
    }

    // 7) Итоговый размер (margin не входит)
    node.layout.size = glm::vec2(
        final_content.x + padding.horizontal() + border.horizontal(),
        final_content.y + padding.vertical() + border.vertical()
    );
}

// --- Основная точка входа для позиционирования ---
void Layout::position_node(Node& node, const glm::vec2& offset) {
    std::visit(
        [&](auto& inner) {
            using T = std::decay_t<decltype(inner)>;
            if constexpr (std::is_same_v<T, Text>) {
                position_text(node, inner, offset);
            } else if constexpr (std::is_same_v<T, Element>) {
                position_element(node, inner, offset);
            }
        },
        node.node_type
    );
}

// --- Позиционирование текста ---
void Layout::position_text(Node& node, Text&, const glm::vec2& offset) {
    // Если есть родитель, используем его для вычисления container_size и align
    if (auto parent_ptr = node.parent.lock()) {
        Node& parent = *parent_ptr;
        Element& p_element = *std::get_if<Element>(&parent.node_type);
        

        BoxSides padding = get_padding(p_element.style);
        BoxSides border  = get_border(p_element.style);

        FlexDirection dir = get_direction(p_element); // направление родителя

        glm::vec2 content_offset = parent.layout.position +
                                   glm::vec2(border.left + padding.left,
                                             border.top  + padding.top);

        glm::vec2 container_size = parent.layout.size -
                                   glm::vec2(border.horizontal() + padding.horizontal(),
                                             border.vertical()   + padding.vertical());

        std::string align_x = p_element.style.get("align-x", "start").asString();
        std::string align_y = p_element.style.get("align-y", "start").asString();

        float x_extra = 0.0f;
        float y_extra = 0.0f;

        // cross-выравнивание по X
        if (dir == FlexDirection::Column) {
            x_extra = cross_offset(container_size.x, node.layout.size.x, align_x);
        } else { // Row
            x_extra = cross_offset(container_size.x, node.layout.size.x, align_x);
        }

        // cross-выравнивание по Y
        if (dir == FlexDirection::Column) {
            y_extra = cross_offset(container_size.y, node.layout.size.y, align_y);
        } else { // Row
            y_extra = cross_offset(container_size.y, node.layout.size.y, align_y);
        }

        node.layout.position = content_offset + glm::vec2(x_extra, y_extra);

    } else {
        // корневая нода
        node.layout.position = offset;
    }

    // Рекурсивно позиционируем детей текста
    for (auto& child : node.children) {
        if (auto* txt = std::get_if<Text>(&child->node_type)) {
            position_text(*child, *txt, node.layout.position);
        } else if (auto* el = std::get_if<Element>(&child->node_type)) {
            position_element(*child, *el, node.layout.position);
        }
    }
}



// --- Позиционирование элемента ---
void Layout::position_element(
    Node& node, Element& element, const glm::vec2& offset
) {
    node.layout.position = offset;

    BoxSides padding = get_padding(element.style);
    BoxSides border  = get_border(element.style);
    FlexDirection dir = get_direction(element);

    glm::vec2 content_offset = offset + glm::vec2(border.left + padding.left,
                                                  border.top  + padding.top);

    std::string align_x = element.style.get("align-x", "start").asString();
    std::string align_y = element.style.get("align-y", "start").asString();

    glm::vec2 container_size = node.layout.size -
        glm::vec2(border.horizontal() + padding.horizontal(),
                  border.vertical()   + padding.vertical());

    // 1) Общий размер вдоль main-оси
    float total_main = 0.0f;
    for (auto& child : node.children) {
        auto& cl = child->get_layout();
        BoxSides cm = get_child_margin(*child);
        total_main += get_child_main_size(cl, cm, dir);
    }

    // 2) Начальная позиция вдоль main
    float container_main = (dir == FlexDirection::Column) ? container_size.y : container_size.x;
    std::string main_align = (dir == FlexDirection::Column) ? align_y : align_x;
    float cursor_main = initial_cursor(container_main, total_main, main_align);

    // 3) Раскладываем детей
    for (auto& child : node.children) {
        auto& cl = child->get_layout();
        BoxSides cm = get_child_margin(*child);

        glm::vec2 child_offset = content_offset;

        if (dir == FlexDirection::Column) {
            float cross_extra = cross_offset(
                container_size.x,
                cl.size.x + cm.horizontal(),
                align_x
            );

            child_offset.x += cm.left + cross_extra;
            child_offset.y += cursor_main + cm.top;

            cursor_main += cl.size.y + cm.vertical();
        } else { // Row
            float cross_extra = cross_offset(
                container_size.y,
                cl.size.y + cm.vertical(),
                align_y
            );

            child_offset.x += cursor_main + cm.left;
            child_offset.y += cm.top + cross_extra;

            cursor_main += cl.size.x + cm.horizontal();
        }

        position_node(*child, child_offset);
    }
}


// -------------------------------------------------------------------------------------

FlexDirection Layout::get_direction(Element& element) {
    std::string dir =
        element.style.get("direction", "column").asString("column");
    if (dir == "row") return FlexDirection::Row;
    return FlexDirection::Column;
}

BoxSides Layout::get_padding(const style::ComputedStyle& style) {
    BoxSides box = BoxSides(style.get("padding", 0).asFloat());

    box.left = style.get("padding-x", box.left).asFloat();
    box.right = style.get("padding-x", box.right).asFloat();

    box.top = style.get("padding-y", box.top).asFloat();
    box.bottom = style.get("padding-y", box.bottom).asFloat();

    box.left = style.get("padding-left", box.left).asFloat();
    box.right = style.get("padding-right", box.right).asFloat();
    box.top = style.get("padding-top", box.top).asFloat();
    box.bottom = style.get("padding-bottom", box.bottom).asFloat();

    return box;
}

BoxSides Layout::get_margin(const style::ComputedStyle& style) {
    BoxSides box = BoxSides(style.get("margin", 0).asFloat());

    box.left = style.get("margin-x", box.left).asFloat();
    box.right = style.get("margin-x", box.right).asFloat();

    box.top = style.get("margin-y", box.top).asFloat();
    box.bottom = style.get("margin-y", box.bottom).asFloat();

    box.left = style.get("margin-left", box.left).asFloat();
    box.right = style.get("margin-right", box.right).asFloat();
    box.top = style.get("margin-top", box.top).asFloat();
    box.bottom = style.get("margin-bottom", box.bottom).asFloat();

    return box;
}

BoxSides Layout::get_border(const style::ComputedStyle& style) {
    BoxSides box = BoxSides(style.get("border-width", 0).asFloat());

    box.left = style.get("border-width-x", box.left).asFloat();
    box.right = style.get("border-width-x", box.right).asFloat();

    box.top = style.get("border-width-y", box.top).asFloat();
    box.bottom = style.get("border-width-y", box.bottom).asFloat();

    box.left = style.get("border-width-left", box.left).asFloat();
    box.right = style.get("border-width-right", box.right).asFloat();
    box.top = style.get("border-width-top", box.top).asFloat();
    box.bottom = style.get("border-width-bottom", box.bottom).asFloat();

    return box;
}

// ----------------------------------------------------------------------
BoxSides Layout::get_child_margin(Node& child) {
    if (auto* child_el = std::get_if<Element>(&child.node_type))
        return get_margin(child_el->style);
    return BoxSides(0.0f);
}

// Общая длина вдоль main-оси (включая margin)
float Layout::get_child_main_size(
    const LayoutBox& cl, const BoxSides& cm, FlexDirection dir
) {
    return (dir == FlexDirection::Column) ? cl.size.y + cm.vertical()
                                          : cl.size.x + cm.horizontal();
}

// Общая ширина/высота вдоль cross-оси (включая margin)
float Layout::get_child_cross_size(
    const LayoutBox& cl, const BoxSides& cm, FlexDirection dir
) {
    return (dir == FlexDirection::Column) ? cl.size.x + cm.horizontal()
                                          : cl.size.y + cm.vertical();
}

// Сдвиг по main-оси в зависимости от align и total_main
float Layout::initial_cursor(
    float container_main, float total_main, const std::string& align
) {
    if (align == "center")
        return std::max(0.0f, (container_main - total_main) * 0.5f);
    else if (align == "end")
        return std::max(0.0f, container_main - total_main);
    return 0.0f;  // start
}

// Смещение по cross-оси для ребёнка
float Layout::cross_offset(
    float container_cross, float child_cross, const std::string& align
) {
    if (align == "center")
        return std::max(0.0f, (container_cross - child_cross) * 0.5f);
    else if (align == "end")
        return std::max(0.0f, container_cross - child_cross);
    return 0.0f;
}
// ----------------------------------------------------------------------