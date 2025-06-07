#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "commons.hpp"

namespace xml {
    class Node;
    class Attribute;
    class Document;

    class Attribute {
        std::string name;
        std::string text;
    public:
        Attribute() = default;
        Attribute(std::string name, std::string text);

        const std::string& getName() const;
        const std::string& getText() const;
        int64_t asInt() const;
        double asFloat() const;
        bool asBool() const;
        glm::vec2 asVec2() const;
        glm::vec3 asVec3() const;
        glm::vec4 asVec4() const;
        glm::vec4 asColor() const;
    };

    /// @brief XML element class. Text element has tag 'text' and attribute
    /// 'text'
    class Node {
        std::string tag;
        std::unordered_map<std::string, Attribute> attrs;
        std::vector<std::unique_ptr<Node>> elements;
    public:
        Node(std::string tag);

        Node(const Node&) = delete;

        /// @brief Add sub-element
        void add(std::unique_ptr<Node> element);

        /// @brief Set attribute value. Creates attribute if does not exists
        /// @param name attribute name
        /// @param text attribute value
        void set(const std::string& name, const std::string& text);

        /// @brief Get element tag
        const std::string& getTag() const;

        bool isText() const {
            return getTag() == "#";
        }

        const std::string& getInnerText() const {
            return attr("#").getText();
        }

        /// @brief Get attribute by name
        /// @param name attribute name
        /// @throws std::runtime_error if element has no attribute
        /// @return xml attribute - {name, value}
        const Attribute& attr(const std::string& name) const;

        /// @brief Get attribute by name
        /// @param name attribute name
        /// @param def default value will be returned wrapped in xmlattribute
        /// if element has no attribute
        /// @return xml attribute - {name, value} or {name, def} if not found
        Attribute attr(const std::string& name, const std::string& def)
            const;

        /// @brief Check if element has attribute
        /// @param name attribute name
        bool has(const std::string& name) const;

        /// @brief Get sub-element by index
        /// @param index sub-element index
        /// @throws std::out_of_range if an invalid index given
        /// @return sub-element
        Node& sub(size_t index);
        const Node& sub(size_t index) const;

        /// @brief Get number of sub-elements
        size_t size() const;

        const std::vector<std::unique_ptr<Node>>& getElements() const;
        const std::unordered_map<std::string, Attribute>& getAttributes() const;
    };

    class Document {
        std::unique_ptr<Node> root = nullptr;
        std::string version;
        std::string encoding;
    public:
        Document(std::string version, std::string encoding);

        void setRoot(std::unique_ptr<Node> element);
        const Node* getRoot() const;

        const std::string& getVersion() const;
        const std::string& getEncoding() const;
    };

    /// @brief Serialize XML Document to string
    /// @param document serializing document
    /// @param nice use human readable format (with indents and line-separators)
    /// @param indentStr indentation characters sequence (default - 4 spaces)
    /// @return XML string
    std::string stringify(
        const Document& document,
        bool nice = true,
        const std::string& indentStr = "    "
    );

    /// @brief Read XML Document from string
    /// @param filename file name will be shown in error messages
    /// @param source xml source code string
    /// @return xml document
    std::unique_ptr<Document> parse(
        std::string_view filename, std::string_view source
    );

    std::unique_ptr<Document> parse_vcm(
        std::string_view filename, std::string_view source, std::string_view tag
    );

    using xmlelement = Node;
}
