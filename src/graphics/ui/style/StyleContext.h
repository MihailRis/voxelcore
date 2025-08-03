// StyleContext.h
#pragma once

#include <string>
#include <vector>
#include <unordered_set>
#include <memory>
#include <glm/glm.hpp>

// Предварительные объявления
namespace style {
    class Stylesheet;
    class ComputedStyle;
}

namespace gui {
    class UINode; // Из предоставленного кода
}

namespace style {

/**
 * Контекст для вычисления стилей.
 * Представляет элемент UI с его характеристиками для применения стилей.
 */
class StyleContext {
public:
    // Типы состояний элемента
    enum class State {
        Normal,
        Hover,
        Active,
        Disabled,
        Focused
    };

private:
    // Базовые свойства элемента для сопоставления с селекторами
    std::string tag_;                           // Тег элемента (например, "button", "panel")
    std::unordered_set<std::string> classes_;   // CSS-классы
    std::string id_;                            // Уникальный идентификатор
    std::unordered_set<State> states_;          // Текущие состояния
    
    // Ссылка на родительский контекст (для наследования)
    const StyleContext* parent_ = nullptr;
    
    // Указатель на соответствующий UINode (для интеграции)
    gui::UINode* ui_node_ = nullptr;

public:
    // Конструкторы
    StyleContext();
    explicit StyleContext(const std::string& tag);
    StyleContext(const std::string& tag, const std::string& id);
    StyleContext(const std::string& tag, const std::vector<std::string>& classes);
    StyleContext(const std::string& tag, const std::string& id, const std::vector<std::string>& classes);

    // Конструктор для интеграции с UINode
    explicit StyleContext(gui::UINode& node);

    // Геттеры
    const std::string& getTag() const { return tag_; }
    const std::string& getID() const { return id_; }
    const std::unordered_set<std::string>& getClasses() const { return classes_; }
    const std::unordered_set<State>& getStates() const { return states_; }
    const StyleContext* getParent() const { return parent_; }
    gui::UINode* getUINode() const { return ui_node_; }

    // Сеттеры
    void setTag(const std::string& tag);
    void setID(const std::string& id);
    void setParent(const StyleContext* parent) { parent_ = parent; }
    void setUINode(gui::UINode* node) { ui_node_ = node; }

    // Работа с классами
    void addClass(const std::string& className);
    void removeClass(const std::string& className);
    bool hasClass(const std::string& className) const;
    void setClasses(const std::vector<std::string>& classes);
    void clearClasses();

    // Работа с состояниями
    void setState(State state, bool enabled = true);
    bool hasState(State state) const;
    void clearStates();

    // Методы для синхронизации с UINode
    void syncWithUINode();
    void updateFromUINode();

    // Утилиты
    std::string getSelectorString() const;
    bool hasID(const std::string& id) const;

    // Создание копии контекста
    std::unique_ptr<StyleContext> clone() const;

    // Операторы сравнения
    bool operator==(const StyleContext& other) const;
    bool operator!=(const StyleContext& other) const { return !(*this == other); }

    // Хэш для использования в контейнерах
    struct Hash {
        std::size_t operator()(const StyleContext& ctx) const;
    };

private:
    // Вспомогательные методы
    void updateSelectorCache() const;
    mutable std::string selector_cache_;
    mutable bool selector_cache_valid_ = false;
};

// Фабричные методы для удобного создания контекстов
namespace context {
    std::unique_ptr<StyleContext> create(const std::string& tag);
    std::unique_ptr<StyleContext> create(const std::string& tag, const std::string& id);
    std::unique_ptr<StyleContext> create(const std::string& tag, 
                                       const std::vector<std::string>& classes);
    std::unique_ptr<StyleContext> create(const std::string& tag, 
                                       const std::string& id, 
                                       const std::vector<std::string>& classes);
    
    // Для интеграции с UINode
    std::unique_ptr<StyleContext> fromUINode(gui::UINode& node);
}

} // namespace style