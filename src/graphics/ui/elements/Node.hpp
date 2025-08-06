#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "coders/xml.hpp"
#include "graphics/ui/style/Stylesheet.h"

using AttributesMap = std::unordered_map<std::string, std::string>;

struct ElementData {
    std::string tag_name;
    AttributesMap attributes;

    ElementData(std::string_view tag, AttributesMap attrs) : tag_name(tag), attributes(std::move(attrs)) {}

    std::optional<std::string> getId() const;
    void setId(std::string_view id);

    std::vector<std::string> getClassList() const;
    void setClassList(const std::vector<std::string>& classes);
    void addClass(std::string_view className);
    void removeClass(std::string_view className);
    bool hasClass(std::string_view className) const;
    void toggleClass(std::string_view className);
};

struct Text {
    std::string content;

    Text(std::string_view text_content) : content(text_content) {
    }
};

struct ElementState {
    bool hover = false;
    bool focus = false;
    bool active = false;
    bool checked = false;
    bool disabled = false;
    size_t nth_child = 0;
    size_t nth_of_type = 0;

    // Оператор для использования в хэш-таблицах
    bool operator==(const ElementState& other) const {
        return hover == other.hover && focus == other.focus &&
               active == other.active && checked == other.checked &&
               disabled == other.disabled && nth_child == other.nth_child &&
               nth_of_type == other.nth_of_type;
    }
};

struct ElementStateHash {
    std::size_t operator()(const ElementState& state) const {
        size_t seed = 0;

        // Простая реализация hash_combine
        auto hash_combine = [](size_t& seed, size_t value) {
            seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        hash_combine(seed, std::hash<bool> {}(state.hover));
        hash_combine(seed, std::hash<bool> {}(state.focus));
        hash_combine(seed, std::hash<bool> {}(state.active));
        hash_combine(seed, std::hash<bool> {}(state.checked));
        hash_combine(seed, std::hash<bool> {}(state.disabled));
        hash_combine(seed, std::hash<size_t> {}(state.nth_child));
        hash_combine(seed, std::hash<size_t> {}(state.nth_of_type));

        return seed;
    }
};

struct Element {
    ElementData data;
    ElementState state;
    style::ComputedStyle style;
    std::unordered_map<ElementState, style::ComputedStyle, ElementStateHash>
        state_styles;

    Element(std::string_view tag, AttributesMap attrs)
        : data(tag, std::move(attrs)) {
    }
};

using NodeType = std::variant<Text, Element>;

struct Node {
    bool root = false;
    std::vector<Node> children;
    NodeType node_type;

    Node(std::string_view text) : node_type(Text(text)) {
    }
    Node(std::string_view tag, AttributesMap attrs)
        : node_type(Element(tag, std::move(attrs))) {
    }
    Node(std::string_view tag, AttributesMap attrs, std::vector<Node> childs)
        : children(std::move(childs)),
          node_type(Element(tag, std::move(attrs))) {
    }

    // Working with childs
    void append_child(Node node);
    void append_childs(std::vector<Node> nodes);
    void prepend_child(Node node);
    void insert_child(size_t index, Node node);
    void remove_child(size_t index);
    void clear_children();

    // Getters
    const std::string& get_tag() const;
    const std::string& get_text() const;
    bool is_text() const;
    bool is_element() const;
    size_t child_count() const;
    const std::vector<Node>& get_children() const;
    bool is_root() const;

    // Working with attributes
    void set_attribute(std::string_view name, std::string_view value);
    void remove_attribute(std::string_view name);
    bool has_attribute(std::string_view name) const;
    std::optional<std::string> get_attribute(std::string_view name) const;
    const AttributesMap& get_attributes() const;

    // Search
    std::vector<Node*> find_by_tag(std::string_view tag);
    std::vector<const Node*> find_by_tag(std::string_view tag) const;
    Node* find_first_by_tag(std::string_view tag);
    const Node* find_first_by_tag(std::string_view tag) const;
    Node* find_by_path(std::string_view path);  // "div>p>span"
    const Node* find_by_path(std::string_view path) const;

    // Parse from XML
    static Node from_xml_string(
        std::string_view filename, std::string_view source
    );
    static Node from_xml_document(const xml::Document& xml_doc);
private:
    static Node from_xml_node(const xml::Node& xml_node);

    Node* find_by_path_impl(
        const std::vector<std::string>& parts, size_t index
    );
    const Node* find_by_path_impl(
        const std::vector<std::string>& parts, size_t index
    ) const;
};
