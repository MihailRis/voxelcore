
#include "Stylesheet.h"
#include "StyleContext.h"
#include <sstream>
#include <regex>
#include <iostream>

namespace style {

PropertyID getPropertyID(const std::string& name) {
    static const std::unordered_map<std::string, PropertyID> property_map = {
        {"color", PropertyID::Color},
        {"background-color", PropertyID::BackgroundColor},
        {"margin", PropertyID::Margin},
        {"margin-top", PropertyID::MarginTop},
        {"margin-right", PropertyID::MarginRight},
        {"margin-bottom", PropertyID::MarginBottom},
        {"margin-left", PropertyID::MarginLeft},
        {"padding", PropertyID::Padding},
        {"padding-top", PropertyID::PaddingTop},
        {"padding-right", PropertyID::PaddingRight},
        {"padding-bottom", PropertyID::PaddingBottom},
        {"padding-left", PropertyID::PaddingLeft},
        {"width", PropertyID::Width},
        {"height", PropertyID::Height},
        {"min-width", PropertyID::MinWidth},
        {"min-height", PropertyID::MinHeight},
        {"max-width", PropertyID::MaxWidth},
        {"max-height", PropertyID::MaxHeight},
        {"z-index", PropertyID::ZIndex},
        {"display", PropertyID::Display},
        {"position", PropertyID::Position},
        {"top", PropertyID::Top},
        {"right", PropertyID::Right},
        {"bottom", PropertyID::Bottom},
        {"left", PropertyID::Left},
        {"opacity", PropertyID::Opacity},
        {"border", PropertyID::Border},
        {"border-radius", PropertyID::BorderRadius},
        {"font-size", PropertyID::FontSize},
        {"font-weight", PropertyID::FontWeight},
        {"text-align", PropertyID::TextAlign}
    };
    
    auto it = property_map.find(name);
    return (it != property_map.end()) ? it->second : PropertyID::Unknown;
}

std::string getPropertyName(PropertyID id) {
    static const std::unordered_map<PropertyID, std::string> property_names = {
        {PropertyID::Color, "color"},
        {PropertyID::BackgroundColor, "background-color"},
        {PropertyID::Margin, "margin"},
        {PropertyID::MarginTop, "margin-top"},
        {PropertyID::MarginRight, "margin-right"},
        {PropertyID::MarginBottom, "margin-bottom"},
        {PropertyID::MarginLeft, "margin-left"},
        {PropertyID::Padding, "padding"},
        {PropertyID::PaddingTop, "padding-top"},
        {PropertyID::PaddingRight, "padding-right"},
        {PropertyID::PaddingBottom, "padding-bottom"},
        {PropertyID::PaddingLeft, "padding-left"},
        {PropertyID::Width, "width"},
        {PropertyID::Height, "height"},
        {PropertyID::MinWidth, "min-width"},
        {PropertyID::MinHeight, "min-height"},
        {PropertyID::MaxWidth, "max-width"},
        {PropertyID::MaxHeight, "max-height"},
        {PropertyID::ZIndex, "z-index"},
        {PropertyID::Display, "display"},
        {PropertyID::Position, "position"},
        {PropertyID::Top, "top"},
        {PropertyID::Right, "right"},
        {PropertyID::Bottom, "bottom"},
        {PropertyID::Left, "left"},
        {PropertyID::Opacity, "opacity"},
        {PropertyID::Border, "border"},
        {PropertyID::BorderRadius, "border-radius"},
        {PropertyID::FontSize, "font-size"},
        {PropertyID::FontWeight, "font-weight"},
        {PropertyID::TextAlign, "text-align"}
    };
    
    auto it = property_names.find(id);
    return (it != property_names.end()) ? it->second : "unknown";
}


Selector::Selector(Type type, std::string value) 
    : type_(type), value_(std::move(value)) {

    switch (type_) {
        case Type::Universal: specificity_ = 0; break;
        case Type::Tag: specificity_ = 1; break;
        case Type::Class: specificity_ = 10; break;
        case Type::ID: specificity_ = 100; break;
    }
}

bool Selector::matches(const StyleContext& context) const {
    switch (type_) {
        case Type::Universal:
            return true;
        case Type::Tag:
            return context.getTag() == value_;
        case Type::Class:
            return context.hasClass(value_);
        case Type::ID:
            return context.hasID(value_);
    }
    return false;
}

bool Selector::operator==(const Selector& other) const {
    return type_ == other.type_ && value_ == other.value_;
}

bool Selector::operator<(const Selector& other) const {
    if (type_ != other.type_) {
        return static_cast<int>(type_) < static_cast<int>(other.type_);
    }
    return value_ < other.value_;
}

std::size_t Selector::Hash::operator()(const Selector& s) const {
    return std::hash<int>{}(static_cast<int>(s.type_)) ^ 
           std::hash<std::string>{}(s.value_);
}


StyleRule::StyleRule(std::vector<Selector> selectors_, 
                     std::unordered_map<PropertyID, value> declarations_)
    : selectors(std::move(selectors_)), declarations(std::move(declarations_)) {
    priority = 0;
}

int StyleRule::getSpecificity() const {
    int total = 0;
    for (const auto& selector : selectors) {
        total += selector.getSpecificity();
    }
    return total;
}

bool StyleRule::matches(const StyleContext& context) const {

    for (const auto& selector : selectors) {
        if (selector.matches(context)) {
            return true;
        }
    }
    return false;
}


void Stylesheet::addRule(const StyleRule& rule) {
    rules_.push_back(rule);
}

void Stylesheet::addRule(const std::vector<Selector>& selectors, 
                        const std::unordered_map<PropertyID, value>& declarations) {
    rules_.emplace_back(selectors, declarations);
}

std::vector<const StyleRule*> Stylesheet::getMatchingRules(const StyleContext& context) const {
    std::vector<const StyleRule*> matching_rules;
    

    if (!tag_index_.empty() || !class_index_.empty() || !id_index_.empty()) {
        std::unordered_set<size_t> candidate_indices;
        
        auto tag_it = tag_index_.find(context.getTag());
        if (tag_it != tag_index_.end()) {
            candidate_indices.insert(tag_it->second.begin(), tag_it->second.end());
        }
        
        for (const auto& cls : context.getClasses()) {
            auto class_it = class_index_.find(cls);
            if (class_it != class_index_.end()) {
                candidate_indices.insert(class_it->second.begin(), class_it->second.end());
            }
        }
        
        auto id_it = id_index_.find(context.getID());
        if (id_it != id_index_.end()) {
            candidate_indices.insert(id_it->second.begin(), id_it->second.end());
        }
        
        for (size_t index : candidate_indices) {
            if (index < rules_.size() && rules_[index].matches(context)) {
                matching_rules.push_back(&rules_[index]);
            }
        }
    } else {

        for (const auto& rule : rules_) {
            if (rule.matches(context)) {
                matching_rules.push_back(&rule);
            }
        }
    }
    
    return matching_rules;
}

void Stylesheet::sortRulesBySpecificity() {
    std::sort(rules_.begin(), rules_.end(), 
              [](const StyleRule& a, const StyleRule& b) {
                  return a.getSpecificity() < b.getSpecificity();
              });
}

void Stylesheet::clear() {
    rules_.clear();
    tag_index_.clear();
    class_index_.clear();
    id_index_.clear();
    selector_cache_.clear();
}

void Stylesheet::buildIndex() {
    tag_index_.clear();
    class_index_.clear();
    id_index_.clear();
    
    for (size_t i = 0; i < rules_.size(); ++i) {
        const auto& rule = rules_[i];
        for (const auto& selector : rule.selectors) {
            switch (selector.getType()) {
                case Selector::Type::Tag:
                    tag_index_[selector.getValue()].push_back(i);
                    break;
                case Selector::Type::Class:
                    class_index_[selector.getValue()].push_back(i);
                    break;
                case Selector::Type::ID:
                    id_index_[selector.getValue()].push_back(i);
                    break;
                case Selector::Type::Universal:

                    break;
            }
        }
    }
}


Selector Stylesheet::createTagSelector(const std::string& tag) {
    return Selector(Selector::Type::Tag, tag);
}

Selector Stylesheet::createClassSelector(const std::string& className) {
    return Selector(Selector::Type::Class, className);
}

Selector Stylesheet::createIDSelector(const std::string& id) {
    return Selector(Selector::Type::ID, id);
}

Selector Stylesheet::createUniversalSelector() {
    return Selector(Selector::Type::Universal, "*");
}


namespace selectors {
    Selector tag(const std::string& name) {
        return Selector(Selector::Type::Tag, name);
    }
    
    Selector cls(const std::string& name) {
        return Selector(Selector::Type::Class, name);
    }
    
    Selector id(const std::string& name) {
        return Selector(Selector::Type::ID, name);
    }
    
    Selector universal() {
        return Selector(Selector::Type::Universal, "*");
    }
}

}