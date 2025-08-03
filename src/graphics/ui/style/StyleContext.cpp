#include "StyleContext.h"
#include "graphics/ui/elements/UINode.hpp"
#include <algorithm>
#include <sstream>

namespace style {


StyleContext::StyleContext() : tag_("node") {}

StyleContext::StyleContext(const std::string& tag) 
    : tag_(tag) {}

StyleContext::StyleContext(const std::string& tag, const std::string& id) 
    : tag_(tag), id_(id) {}

StyleContext::StyleContext(const std::string& tag, const std::vector<std::string>& classes) 
    : tag_(tag) {
    for (const auto& cls : classes) {
        classes_.insert(cls);
    }
}

StyleContext::StyleContext(const std::string& tag, const std::string& id, const std::vector<std::string>& classes) 
    : tag_(tag), id_(id) {
    for (const auto& cls : classes) {
        classes_.insert(cls);
    }
}

StyleContext::StyleContext(gui::UINode& node) 
    : tag_("node"), ui_node_(&node) {
    updateFromUINode();
}


void StyleContext::setTag(const std::string& tag) {
    tag_ = tag;
    selector_cache_valid_ = false;
}

void StyleContext::setID(const std::string& id) {
    id_ = id;
    selector_cache_valid_ = false;
}


void StyleContext::addClass(const std::string& className) {
    classes_.insert(className);
    selector_cache_valid_ = false;
}

void StyleContext::removeClass(const std::string& className) {
    classes_.erase(className);
    selector_cache_valid_ = false;
}

bool StyleContext::hasClass(const std::string& className) const {
    return classes_.find(className) != classes_.end();
}

void StyleContext::setClasses(const std::vector<std::string>& classes) {
    classes_.clear();
    for (const auto& cls : classes) {
        classes_.insert(cls);
    }
    selector_cache_valid_ = false;
}

void StyleContext::clearClasses() {
    classes_.clear();
    selector_cache_valid_ = false;
}


void StyleContext::setState(State state, bool enabled) {
    if (enabled) {
        states_.insert(state);
    } else {
        states_.erase(state);
    }
}

bool StyleContext::hasState(State state) const {
    return states_.find(state) != states_.end();
}

void StyleContext::clearStates() {
    states_.clear();
}


void StyleContext::syncWithUINode() {
    if (ui_node_) {
        updateFromUINode();
    }
}

void StyleContext::updateFromUINode() {
    if (!ui_node_) return;

    
    id_ = ui_node_->getId();
    
    
    std::string classname = ui_node_->getClassname();
    if (!classname.empty()) {
        
        std::istringstream iss(classname);
        std::string cls;
        classes_.clear();
        while (iss >> cls) {
            if (!cls.empty()) {
                classes_.insert(cls);
            }
        }
    }
    
    
    states_.clear();
    if (!ui_node_->isEnabled()) {
        states_.insert(State::Disabled);
    }
    if (ui_node_->isHover()) {
        states_.insert(State::Hover);
    }
    if (ui_node_->isPressed()) {
        states_.insert(State::Active);
    }
    if (ui_node_->isFocused()) {
        states_.insert(State::Focused);
    }
    
    selector_cache_valid_ = false;
}


std::string StyleContext::getSelectorString() const {
    if (selector_cache_valid_) {
        return selector_cache_;
    }

    std::ostringstream oss;
    
    
    if (!tag_.empty()) {
        oss << tag_;
    }
    
    
    if (!id_.empty()) {
        oss << "#" << id_;
    }
    
    
    for (const auto& cls : classes_) {
        oss << "." << cls;
    }
    
    selector_cache_ = oss.str();
    selector_cache_valid_ = true;
    
    return selector_cache_;
}

bool StyleContext::hasID(const std::string& id) const {
    return id == id;
}

std::unique_ptr<StyleContext> StyleContext::clone() const {
    auto clone = std::make_unique<StyleContext>(tag_, id_);
    clone->classes_ = classes_;
    clone->states_ = states_;
    clone->parent_ = parent_;
    clone->ui_node_ = ui_node_;
    clone->selector_cache_ = selector_cache_;
    clone->selector_cache_valid_ = selector_cache_valid_;
    return clone;
}

bool StyleContext::operator==(const StyleContext& other) const {
    return tag_ == other.tag_ &&
           id_ == other.id_ &&
           classes_ == other.classes_ &&
           states_ == other.states_;
}

std::size_t StyleContext::Hash::operator()(const StyleContext& ctx) const {
    std::size_t h1 = std::hash<std::string>{}(ctx.tag_);
    std::size_t h2 = std::hash<std::string>{}(ctx.id_);
    
    return h1 ^ (h2 << 1);
}

void StyleContext::updateSelectorCache() const {
    
    selector_cache_valid_ = false;
}


namespace context {
    std::unique_ptr<StyleContext> create(const std::string& tag) {
        return std::make_unique<StyleContext>(tag);
    }
    
    std::unique_ptr<StyleContext> create(const std::string& tag, const std::string& id) {
        return std::make_unique<StyleContext>(tag, id);
    }
    
    std::unique_ptr<StyleContext> create(const std::string& tag, 
                                       const std::vector<std::string>& classes) {
        return std::make_unique<StyleContext>(tag, classes);
    }
    
    std::unique_ptr<StyleContext> create(const std::string& tag, 
                                       const std::string& id, 
                                       const std::vector<std::string>& classes) {
        return std::make_unique<StyleContext>(tag, id, classes);
    }
    
    std::unique_ptr<StyleContext> fromUINode(gui::UINode& node) {
        return std::make_unique<StyleContext>(node);
    }
}

} 