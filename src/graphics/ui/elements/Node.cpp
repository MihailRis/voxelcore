#pragma once

#include "Node.hpp"

#include <sstream>
#include <variant>

#include "../layout/Layout.hpp"
#include "constants.hpp"
#include "util/stringutil.hpp"

std::shared_ptr<Node> Node::from_xml_node(const xml::Node& xml_node) {
    if (xml_node.isText()) {
        return std::make_shared<Node>(xml_node.getInnerText());
    }

    AttributesMap attrs;
    const auto& xml_attrs = xml_node.getAttributes();
    for (const auto& [name, attr] : xml_attrs) {
        attrs[name] = attr.getText();
    }

    auto dom_node = std::make_shared<Node>(xml_node.getTag(), std::move(attrs));
    dom_node->root = true;

    for (size_t i = 0; i < xml_node.size(); ++i) {
        const xml::Node& child = xml_node.sub(i);
        auto child_node = from_xml_node(child);
        child_node->parent = dom_node;   // безопасно, shared_from_this не нужен здесь
        dom_node->children.push_back(std::move(child_node));
    }

    return dom_node;
}

std::shared_ptr<Node> Node::from_xml_document(const xml::Document& xml_doc) {
    if (!xml_doc.getRoot()) {
        throw std::runtime_error("XML document has no root element");
    }

    return from_xml_node(*xml_doc.getRoot());
}

std::shared_ptr<Node> Node::from_xml_string(std::string_view filename, std::string_view source) {
    auto xml_doc = xml::parse(filename, source);
    return from_xml_document(*xml_doc);
}

// Working with childs
//

void Node::append_child(Node node) {
    auto child_ptr =
        std::make_shared<Node>(std::move(node));
    child_ptr->parent = shared_from_this();
    children.push_back(std::move(child_ptr));     // добавляем в children
}

void Node::prepend_child(Node node) {
    auto child_ptr =
        std::make_unique<Node>(std::move(node));  // создаём unique_ptr
    child_ptr->parent = shared_from_this();
    children.insert(children.begin(), std::move(child_ptr));
}

void Node::append_childs(std::vector<Node> nodes) {
    children.reserve(children.size() + nodes.size());

    for (Node node : nodes) {
        auto child_ptr =
            std::make_unique<Node>(std::move(node));  // создаём unique_ptr
        child_ptr->parent = shared_from_this();
        children.push_back(std::move(child_ptr));
    }
}

void Node::insert_child(size_t index, Node node) {
    if (index > children.size()) {
        throw std::out_of_range("Index out of range");
    }
    auto child_ptr =
        std::make_unique<Node>(std::move(node));  // создаём unique_ptr
    child_ptr->parent = shared_from_this();
    children.insert(children.begin() + index, std::move(child_ptr));
}

void Node::remove_child(size_t index) {
    if (index >= children.size()) {
        throw std::out_of_range("Index out of range");
    }
    children.erase(children.begin() + index);
}

void Node::clear_children() {
    children.clear();
}

const std::string& Node::get_tag() const {
    return std::visit(
        [](const auto& node) -> const std::string& {
            using T = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<T, Text>) {
                static const std::string text_tag = "#text";
                return text_tag;
            } else {
                return node.data.tag_name;
            }
        },
        node_type
    );
}

const std::string& Node::get_text() const {
    return std::visit(
        [](const auto& node) -> const std::string& {
            using T = std::decay_t<decltype(node)>;
            if constexpr (std::is_same_v<T, Text>) {
                return node.content;
            } else {
                static const std::string empty_string = "";
                return empty_string;
            }
        },
        node_type
    );
}

bool Node::is_text() const {
    return std::holds_alternative<Text>(node_type);
}

bool Node::is_element() const {
    return std::holds_alternative<Element>(node_type);
}

size_t Node::child_count() const {
    return children.size();
}

void Node::set_attribute(std::string_view name, std::string_view value) {
    if (auto* element = std::get_if<Element>(&node_type)) {
        element->data.attributes[std::string(name)] = std::string(value);
    } else {
        throw std::runtime_error("Cannot set attribute on text node");
    }
}

void Node::remove_attribute(std::string_view name) {
    if (auto* element = std::get_if<Element>(&node_type)) {
        element->data.attributes.erase(std::string(name));
    }
}

bool Node::has_attribute(std::string_view name) const {
    if (const auto* element = std::get_if<Element>(&node_type)) {
        return element->data.attributes.find(std::string(name)) !=
               element->data.attributes.end();
    }
    return false;
}

std::optional<std::string> Node::get_attribute(std::string_view name) const {
    if (const auto* element = std::get_if<Element>(&node_type)) {
        auto it = element->data.attributes.find(std::string(name));
        if (it != element->data.attributes.end()) {
            return it->second;
        }
    }
    return std::nullopt;
}

const AttributesMap& Node::get_attributes() const {
    static const AttributesMap empty_attrs = {};

    if (const auto* element = std::get_if<Element>(&node_type)) {
        return element->data.attributes;
    }
    return empty_attrs;
}

std::vector<Node*> Node::find_by_tag(std::string_view tag) {
    std::vector<Node*> result;

    if (is_element() && get_tag() == tag) {
        result.push_back(this);
    }

    for (auto& child : children) {
        auto child_results = child->find_by_tag(tag);
        result.insert(result.end(), child_results.begin(), child_results.end());
    }

    return result;
}

std::vector<const Node*> Node::find_by_tag(std::string_view tag) const {
    std::vector<const Node*> result;

    if (is_element() && get_tag() == tag) {
        result.push_back(this);
    }

    for (const auto& child : children) {
        auto child_results = child->find_by_tag(tag);
        result.insert(result.end(), child_results.begin(), child_results.end());
    }

    return result;
}

Node* Node::find_first_by_tag(std::string_view tag) {
    if (is_element() && get_tag() == tag) {
        return this;
    }

    for (auto& child : children) {
        if (auto* found = child->find_first_by_tag(tag)) {
            return found;
        }
    }

    return nullptr;
}

const Node* Node::find_first_by_tag(std::string_view tag) const {
    if (is_element() && get_tag() == tag) {
        return this;
    }

    for (const auto& child : children) {
        if (const auto* found = child->find_first_by_tag(tag)) {
            return found;
        }
    }

    return nullptr;
}

Node* Node::find_by_path(std::string_view path) {
    if (path.empty()) {
        return nullptr;
    }

    std::string path_str(path);
    std::vector<std::string> parts;
    size_t start = 0;
    size_t pos = 0;

    while ((pos = path_str.find('>', start)) != std::string::npos) {
        std::string part = path_str.substr(start, pos - start);
        if (!part.empty()) {
            parts.push_back(std::move(part));
        }
        start = pos + 1;
    }

    std::string last_part = path_str.substr(start);
    if (!last_part.empty()) {
        parts.push_back(std::move(last_part));
    }

    if (parts.empty()) {
        return nullptr;
    }

    return find_by_path_impl(parts, 0);
}

const Node* Node::find_by_path(std::string_view path) const {
    if (path.empty()) {
        return nullptr;
    }

    std::string path_str(path);
    std::vector<std::string> parts;
    size_t start = 0;
    size_t pos = 0;

    while ((pos = path_str.find('>', start)) != std::string::npos) {
        std::string part = path_str.substr(start, pos - start);
        if (!part.empty()) {
            parts.push_back(std::move(part));
        }
        start = pos + 1;
    }

    std::string last_part = path_str.substr(start);
    if (!last_part.empty()) {
        parts.push_back(std::move(last_part));
    }

    if (parts.empty()) {
        return nullptr;
    }

    return find_by_path_impl(parts, 0);
}

Node* Node::find_by_path_impl(
    const std::vector<std::string>& parts, size_t index
) {
    if (index >= parts.size()) {
        return nullptr;
    }

    const std::string& expected_tag = parts[index];

    if (is_element() && get_tag() == expected_tag) {
        if (index == parts.size() - 1) {
            return this;
        }

        for (auto& child : children) {
            if (auto* found = child->find_by_path_impl(parts, index + 1)) {
                return found;
            }
        }
    }

    return nullptr;
}

const Node* Node::find_by_path_impl(
    const std::vector<std::string>& parts, size_t index
) const {
    if (index >= parts.size()) {
        return nullptr;
    }

    const std::string& expected_tag = parts[index];

    if (is_element() && get_tag() == expected_tag) {
        if (index == parts.size() - 1) {
            return this;
        }

        for (const auto& child : children) {
            if (const auto* found = child->find_by_path_impl(parts, index + 1)) {
                return found;
            }
        }
    }

    return nullptr;
}

const std::vector<std::shared_ptr<Node>>& Node::get_children() const {
    return children;
}

bool Node::is_root() const {
    return root;
}

// Получение ID
std::optional<std::string> ElementData::getId() const {
    auto it = attributes.find("id");
    if (it != attributes.end()) {
        return it->second;
    }
    return std::nullopt;
}

// Установка ID
void ElementData::setId(std::string_view id) {
    if (id.empty()) {
        attributes.erase("id");
    } else {
        attributes["id"] = std::string(id);
    }
}

// Получение списка классов
std::vector<std::string> ElementData::getClassList() const {
    std::vector<std::string> classes;
    auto it = attributes.find("class");
    if (it != attributes.end() && !it->second.empty()) {
        std::istringstream iss(it->second);
        std::string cls;
        while (iss >> cls) {
            if (!cls.empty()) {
                classes.push_back(cls);
            }
        }
    }
    return classes;
}

// Установка списка классов
void ElementData::setClassList(const std::vector<std::string>& classes) {
    if (classes.empty()) {
        attributes.erase("class");
        return;
    }

    std::string class_string;
    for (size_t i = 0; i < classes.size(); ++i) {
        if (i > 0) {
            class_string += " ";
        }
        class_string += classes[i];
    }
    attributes["class"] = class_string;
}

// Добавление класса
void ElementData::addClass(std::string_view className) {
    if (className.empty()) return;

    auto classes = getClassList();
    if (std::find(classes.begin(), classes.end(), className) == classes.end()) {
        classes.push_back(std::string(className));
        setClassList(classes);
    }
}

// Удаление класса
void ElementData::removeClass(std::string_view className) {
    if (className.empty()) return;

    auto classes = getClassList();
    auto it = std::find(classes.begin(), classes.end(), className);
    if (it != classes.end()) {
        classes.erase(it);
        setClassList(classes);
    }
}

// Проверка наличия класса
bool ElementData::hasClass(std::string_view className) const {
    if (className.empty()) return false;

    auto classes = getClassList();
    return std::find(classes.begin(), classes.end(), className) !=
           classes.end();
}

// Переключение класса
void ElementData::toggleClass(std::string_view className) {
    if (className.empty()) return;

    if (hasClass(className)) {
        removeClass(className);
    } else {
        addClass(className);
    }
}

void Node::draw(const DrawContext& ctx, const Assets& assets) {
    LayoutContext context;
    context.available_space = ctx.getViewport();
    context.parent_layout = nullptr;

    Layout::compute_layout(*this, context);

    draw(ctx, assets, context);
}

void Node::draw(
    const DrawContext& ctx, const Assets& assets, const LayoutContext& context
) {
    auto batch = ctx.getBatch2D();
    
    // Создаём под-контекст для локального управления состоянием
    DrawContext childCtx = ctx.sub();

    // Определяем viewport/скиссоры, если нужно (для примера оставим полный размер)
    // childCtx.setScissors(glm::vec4(layout.x, layout.y, layout.width, layout.height));

    // Сбрасываем батч перед рисованием текущей ноды
    batch->flush();
    batch->untexture();

    std::visit(
        [this, &childCtx, &assets](const auto& node_type) {
            using T = std::decay_t<decltype(node_type)>;
            if constexpr (std::is_same_v<T, Text>) {
                draw_text(childCtx, assets, node_type);
            } else if constexpr (std::is_same_v<T, Element>) {
                draw_element(childCtx, assets, node_type);
            }
        },
        node_type
    );

    // Рекурсивно рисуем детей в отдельном под-контексте
    for (auto& child : children) {
        child->draw(childCtx, assets, context);
    }

    // Сбрасываем батч после рисования всех детей, чтобы гарантировать корректное состояние OpenGL
    batch->flush();
    batch->untexture();
}

void Node::draw_text(const DrawContext& ctx, const Assets& assets, Text text) {
    auto batch = ctx.getBatch2D();

    batch->flush();
    batch->untexture();

    auto font = assets.get<Font>(FONT_DEFAULT);

    batch->setColor(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    font->draw(
        *batch,
        util::str2wstr_utf8("A"),
        layout.x,
        layout.y,
        text.styles.get(),
        0
    );
}

void Node::draw_element(
    const DrawContext& ctx, const Assets& assets, Element element
) {
    auto batch = ctx.getBatch2D();

    batch->flush();
    batch->untexture();

    batch->setColor(element.style.get("background", "#ff0000").asColor());
    batch->rect(layout.x, layout.y, layout.width, layout.height);
}
