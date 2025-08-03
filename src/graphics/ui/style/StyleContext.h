
#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace style {
    class Stylesheet;
    class ComputedStyle;
}

namespace gui {
    class UINode;
}

namespace style {

    class StyleContext {
    public:
        enum class State { Normal, Hover, Active, Disabled, Focused };
    private:
        std::string tag_;
        std::unordered_set<std::string> classes_;
        std::string id_;
        std::unordered_set<State> states_;

        const StyleContext* parent_ = nullptr;

        gui::UINode* ui_node_ = nullptr;
    public:
        StyleContext();
        explicit StyleContext(const std::string& tag);
        StyleContext(const std::string& tag, const std::string& id);
        StyleContext(
            const std::string& tag, const std::vector<std::string>& classes
        );
        StyleContext(
            const std::string& tag,
            const std::string& id,
            const std::vector<std::string>& classes
        );

        explicit StyleContext(gui::UINode& node);

        const std::string& getTag() const {
            return tag_;
        }
        const std::string& getID() const {
            return id_;
        }
        const std::unordered_set<std::string>& getClasses() const {
            return classes_;
        }
        const std::unordered_set<State>& getStates() const {
            return states_;
        }
        const StyleContext* getParent() const {
            return parent_;
        }
        gui::UINode* getUINode() const {
            return ui_node_;
        }

        void setTag(const std::string& tag);
        void setID(const std::string& id);
        void setParent(const StyleContext* parent) {
            parent_ = parent;
        }
        void setUINode(gui::UINode* node) {
            ui_node_ = node;
        }

        void addClass(const std::string& className);
        void removeClass(const std::string& className);
        bool hasClass(const std::string& className) const;
        void setClasses(const std::vector<std::string>& classes);
        void clearClasses();

        void setState(State state, bool enabled = true);
        bool hasState(State state) const;
        void clearStates();

        void syncWithUINode();
        void updateFromUINode();

        std::string getSelectorString() const;
        bool hasID(const std::string& id) const;

        std::unique_ptr<StyleContext> clone() const;

        bool operator==(const StyleContext& other) const;
        bool operator!=(const StyleContext& other) const {
            return !(*this == other);
        }

        struct Hash {
            std::size_t operator()(const StyleContext& ctx) const;
        };
    private:
        void updateSelectorCache() const;
        mutable std::string selector_cache_;
        mutable bool selector_cache_valid_ = false;
    };

    namespace context {
        std::unique_ptr<StyleContext> create(const std::string& tag);
        std::unique_ptr<StyleContext> create(
            const std::string& tag, const std::string& id
        );
        std::unique_ptr<StyleContext> create(
            const std::string& tag, const std::vector<std::string>& classes
        );
        std::unique_ptr<StyleContext> create(
            const std::string& tag,
            const std::string& id,
            const std::vector<std::string>& classes
        );

        std::unique_ptr<StyleContext> fromUINode(gui::UINode& node);
    }

}