#pragma once

#include "Node.hpp"

Node Node::from_xml_node(const xml::Node& xml_node) {

    if (xml_node.isText()) {
        return Node(xml_node.getInnerText());
    }
    
    AttributesMap attrs;
    const auto& xml_attrs = xml_node.getAttributes();
    for (const auto& [name, attr] : xml_attrs) {
        attrs[name] = attr.getText();
    }
    
    Node dom_node(xml_node.getTag(), std::move(attrs));
    
    for (size_t i = 0; i < xml_node.size(); ++i) {
        const xml::Node& child = xml_node.sub(i);
        dom_node.append_child(from_xml_node(child));
    }
    
    return dom_node;
}

Node Node::from_xml_document(const xml::Document& xml_doc) {
    if (!xml_doc.getRoot()) {
        throw std::runtime_error("XML document has no root element");
    }
    
    return from_xml_node(*xml_doc.getRoot());
}

Node Node::from_xml_string(std::string_view filename, std::string_view source) {
    auto xml_doc = xml::parse(filename, source);
    return from_xml_document(*xml_doc);
}

// Working with childs
//

void Node::append_child(Node node) {
    children.push_back( std::move( node ) );
}

void Node::prepend_child(Node node) {
    children.insert( children.begin(), std::move( node ) );
}

void Node::append_childs(std::vector<Node> nodes) {
    children.reserve( children.size() + nodes.size() );

    for ( Node node: nodes ) {
        children.push_back( std::move( node ) );
    }
}

void Node::insert_child(size_t index, Node node) {
    if (index > children.size()) {
        throw std::out_of_range("Index out of range");
    }
    children.insert(children.begin() + index, std::move(node));
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
    return std::visit([](const auto& node) -> const std::string& {
        using T = std::decay_t<decltype(node)>;
        if constexpr (std::is_same_v<T, Text>) {
            static const std::string text_tag = "#text";
            return text_tag;
        } else {
            return node.data.tag_name;
        }
    }, node_type);
}

const std::string& Node::get_text() const {
    return std::visit([](const auto& node) -> const std::string& {
        using T = std::decay_t<decltype(node)>;
        if constexpr (std::is_same_v<T, Text>) {
            return node.content;
        } else {
            static const std::string empty_string = "";
            return empty_string;
        }
    }, node_type);
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
        return element->data.attributes.find(std::string(name)) != element->data.attributes.end();
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
        auto child_results = child.find_by_tag(tag);
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
        auto child_results = child.find_by_tag(tag);
        result.insert(result.end(), child_results.begin(), child_results.end());
    }
    
    return result;
}

Node* Node::find_first_by_tag(std::string_view tag) {
    if (is_element() && get_tag() == tag) {
        return this;
    }
    
    for (auto& child : children) {
        if (auto* found = child.find_first_by_tag(tag)) {
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
        if (const auto* found = child.find_first_by_tag(tag)) {
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

Node* Node::find_by_path_impl(const std::vector<std::string>& parts, size_t index) {
    if (index >= parts.size()) {
        return nullptr;
    }
    
    const std::string& expected_tag = parts[index];
    
    if (is_element() && get_tag() == expected_tag) {
        if (index == parts.size() - 1) {
            return this;
        }
        
        for (auto& child : children) {
            if (auto* found = child.find_by_path_impl(parts, index + 1)) {
                return found;
            }
        }
    }
    
    return nullptr;
}

const Node* Node::find_by_path_impl(const std::vector<std::string>& parts, size_t index) const {
    if (index >= parts.size()) {
        return nullptr;
    }
    
    const std::string& expected_tag = parts[index];
    
    if (is_element() && get_tag() == expected_tag) {
        if (index == parts.size() - 1) {
            return this;
        }
        
        for (const auto& child : children) {
            if (const auto* found = child.find_by_path_impl(parts, index + 1)) {
                return found;
            }
        }
    }
    
    return nullptr;
}

const std::vector<Node>& Node::get_children() const {
    return children;
}