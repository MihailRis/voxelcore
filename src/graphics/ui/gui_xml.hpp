#pragma once

#include "GUI.hpp"
#include "coders/xml.hpp"

#include <memory>
#include <stack>
#include <unordered_set>
#include <unordered_map>

namespace gui {
    class UiXmlReader;

    using uinode_reader = std::function<
        std::shared_ptr<UINode>(UiXmlReader&, const xml::xmlelement&)>;

    class UiXmlReader {
        gui::GUI& gui;
        std::unordered_map<std::string, uinode_reader> readers;
        std::unordered_set<std::string> ignored;
        std::stack<std::string> contextStack;
        std::string filename;
        scriptenv env;
    public:
        UiXmlReader(gui::GUI& gui, scriptenv&& env);

        void add(const std::string& tag, uinode_reader reader);
        bool hasReader(const std::string& tag) const;
        void addIgnore(const std::string& tag);
        
        std::shared_ptr<UINode> readUINode(const xml::xmlelement& element);
        
        void readUINode(
            const UiXmlReader& reader, 
            const xml::xmlelement& element,
            UINode& node
        );

        void readUINode(
            UiXmlReader& reader, 
            const xml::xmlelement& element,
            Container& container
        );

        std::shared_ptr<UINode> readXML(
            const std::string& filename,
            const std::string& source
        );

        std::shared_ptr<UINode> readXML(
            const std::string& filename,
            const xml::xmlelement& root
        );

        const std::string& getContext() const;
        const scriptenv& getEnvironment() const;
        const std::string& getFilename() const;
        gui::GUI& getGUI() const;
    };
}
