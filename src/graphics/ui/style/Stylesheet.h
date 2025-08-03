#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include "value.h"

namespace style {

class StyleContext;
class ComputedStyle;
class Selector;

enum class PropertyID : uint32_t {
    Unknown = 0,
    Color,
    BackgroundColor,
    Margin,
    MarginTop,
    MarginRight,
    MarginBottom,
    MarginLeft,
    Padding,
    PaddingTop,
    PaddingRight,
    PaddingBottom,
    PaddingLeft,
    Width,
    Height,
    MinWidth,
    MinHeight,
    MaxWidth,
    MaxHeight,
    ZIndex,
    Display,
    Position,
    Top,
    Right,
    Bottom,
    Left,
    Opacity,
    Border,
    BorderRadius,
    FontSize,
    FontWeight,
    TextAlign,
};

// Получение PropertyID по строке
PropertyID getPropertyID(const std::string& name);
std::string getPropertyName(PropertyID id);

// Селектор стиля
class Selector {
public:
    enum class Type {
        Tag,        // block
        Class,      // .highlight
        ID,         // #main
        Universal   // *
    };

private:
    Type type_;
    std::string value_;
    int specificity_ = 0;

public:
    Selector(Type type, std::string value);
    
    Type getType() const { return type_; }
    const std::string& getValue() const { return value_; }
    int getSpecificity() const { return specificity_; }
    
    // Проверяет, соответствует ли селектор контексту
    bool matches(const StyleContext& context) const;
    
    // Операторы сравнения
    bool operator==(const Selector& other) const;
    bool operator<(const Selector& other) const;
    
    struct Hash {
        std::size_t operator()(const Selector& s) const;
    };
};

struct StyleRule {
    std::vector<Selector> selectors;
    std::unordered_map<PropertyID, value> declarations;
    int priority = 0; // для !important и других приоритетов
    
    StyleRule() = default;
    StyleRule(std::vector<Selector> selectors_, 
              std::unordered_map<PropertyID, value> declarations_);
    
    // Вычисляет специфичность правила
    int getSpecificity() const;
    
    // Проверяет, подходит ли правило для контекста
    bool matches(const StyleContext& context) const;
};

class Stylesheet {
private:
    std::vector<StyleRule> rules_;
    
    // Индексы для быстрого поиска
    std::unordered_map<std::string, std::vector<size_t>> tag_index_;
    std::unordered_map<std::string, std::vector<size_t>> class_index_;
    std::unordered_map<std::string, std::vector<size_t>> id_index_;
    
    // Кэш для часто используемых селекторов
    mutable std::unordered_map<std::string, std::shared_ptr<Selector>> selector_cache_;

public:
    Stylesheet() = default;

    void addRule(const StyleRule& rule);
    void addRule(const std::vector<Selector>& selectors, 
                 const std::unordered_map<PropertyID, value>& declarations);
    
    bool parseAndAddRule(const std::string& css_string);
    
    std::vector<const StyleRule*> getMatchingRules(const StyleContext& context) const;
    
    // Сортировка правил по специфичности (для каскада)
    void sortRulesBySpecificity();
    
    void clear();
    
    // Получение количества правил
    size_t getRuleCount() const { return rules_.size(); }
    
    // Индексация для ускорения поиска
    void buildIndex();
    
    // Вспомогательные методы для создания селекторов
    static Selector createTagSelector(const std::string& tag);
    static Selector createClassSelector(const std::string& className);
    static Selector createIDSelector(const std::string& id);
    static Selector createUniversalSelector();

private:
    // Вспомогательные методы для парсинга
    std::vector<Selector> parseSelectors(const std::string& selector_string);
    std::unordered_map<PropertyID, value> parseDeclarations(const std::string& declarations_string);
    
    // Получение или создание селектора с кэшированием
    std::shared_ptr<Selector> getOrCreateSelector(Selector::Type type, const std::string& value) const;
};

namespace selectors {
    Selector tag(const std::string& name);
    Selector cls(const std::string& name);  // class
    Selector id(const std::string& name);
    Selector universal();

    std::vector<Selector> combine(const std::vector<Selector>& a, const std::vector<Selector>& b);
}

}