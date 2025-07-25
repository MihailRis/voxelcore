#pragma once

#include "delegates.hpp"
#include "graphics/core/commons.hpp"
#include "window/input.hpp"

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <unordered_map>

class DrawContext;
class Assets;

namespace gui {
    class UINode;
    class GUI;
    class Container;

    using onaction = std::function<void(GUI&)>;
    using onnumberchange = std::function<void(GUI&, double)>;
    using onstringchange = std::function<void(GUI&, const std::string&)>;

    template<typename... Args>
    class CallbacksSet {
    public:
        using Func = std::function<void(Args...)>;
    private:
        std::unique_ptr<std::vector<Func>> callbacks;
    public:
        void listen(const Func& callback) {
            if (callbacks == nullptr) {
                callbacks = std::make_unique<std::vector<Func>>();
            }
            callbacks->push_back(callback);
        }

        void notify(Args&&... args) {
            if (callbacks) {
                for (auto& callback : *callbacks) {
                    callback(std::forward<Args>(args)...);
                }
            }
        }
    };

    using ActionsSet = CallbacksSet<GUI&>;
    using StringCallbacksSet = CallbacksSet<GUI&, const std::string&>;
    
    enum class Align {
        left, center, right,
        top=left, bottom=right,
    };

    enum class Gravity {
        none,
        
        top_left,
        top_center,
        top_right,

        center_left,
        center_center,
        center_right,

        bottom_left,
        bottom_center,
        bottom_right
    };

    /// @brief Base abstract class for all UI elements
    class UINode : public std::enable_shared_from_this<UINode> {
    protected:
        GUI& gui;
        bool mustRefresh = true;
    private:
        /// @brief element identifier used for direct access in UiDocument
        std::string id = "";
        /// @brief element enabled state
        bool enabled = true;
    protected:
        /// @brief element position within the parent element
        glm::vec2 pos {0.0f};
        /// @brief element size (width, height)
        glm::vec2 size;
        /// @brief minimal element size
        glm::vec2 minSize {1.0f};
        /// @brief maximal element size
        glm::vec2 maxSize {1e6f};
        /// @brief element primary color (background-color or text-color if label)
        glm::vec4 color {1.0f};
        /// @brief element color when mouse is over it
        glm::vec4 hoverColor {1.0f};
        /// @brief element color when clicked
        glm::vec4 pressedColor {1.0f};
        /// @brief element margin (only supported for Panel sub-nodes)
        glm::vec4 margin {0.0f};
        /// @brief is element visible
        bool visible = true;
        /// @brief is mouse over the element
        bool hover = false;
        /// @brief is mouse has been pressed over the element and not released yet
        bool pressed = false;
        /// @brief is element focused
        bool focused = false;
        /// @brief is element opaque for cursor interaction
        bool interactive = true;
        /// @brief does the element support resizing by parent elements
        bool resizing = true;
        /// @brief z-index property specifies the stack order of an element
        int zindex = 0;
        /// @brief element content alignment (supported by Label only)
        Align align = Align::left;
        /// @brief parent element
        UINode* parent = nullptr;
        /// @brief position supplier for the element (called on parent element size update)
        vec2supplier positionfunc = nullptr;
        /// @brief size supplier for the element (called on parent element size update)
        vec2supplier sizefunc = nullptr;
        /// @brief 'onclick' callbacks
        ActionsSet actions;
        /// @brief 'ondoubleclick' callbacks
        ActionsSet doubleClickCallbacks;
        /// @brief 'onfocus' callbacks
        ActionsSet focusCallbacks;
        /// @brief 'ondefocus' callbacks
        ActionsSet defocusCallbacks;
        /// @brief element tooltip text
        std::wstring tooltip;
        /// @brief element tooltip delay
        float tooltipDelay = 0.5f;
        /// @brief cursor shape when mouse is over the element
        CursorShape cursor = CursorShape::ARROW;

        UINode(GUI& gui, glm::vec2 size);
    public:
        virtual ~UINode();

        /// @brief Called every frame for all visible elements 
        /// @param delta delta timУ
        virtual void act(float delta) {
            if (mustRefresh) {
                mustRefresh = false;
                refresh();
            }
        };
        virtual void draw(const DrawContext& pctx, const Assets& assets) = 0;

        virtual void setVisible(bool flag);
        bool isVisible() const;

        virtual void setEnabled(bool flag);
        bool isEnabled() const;

        virtual void setAlign(Align align);
        Align getAlign() const;

        virtual void setHover(bool flag);
        bool isHover() const;

        virtual void setParent(UINode* node);
        UINode* getParent() const;

        /// @brief Set element color (doesn't affect inner elements).
        /// Also replaces hover color to avoid adding extra properties
        virtual void setColor(glm::vec4 newColor);

        /// @brief Get element color 
        /// @return (float R,G,B,A in range [0.0, 1.0])
        glm::vec4 getColor() const;

        virtual void setHoverColor(glm::vec4 newColor);
        glm::vec4 getHoverColor() const;

        virtual glm::vec4 getPressedColor() const;
        virtual void setPressedColor(glm::vec4 color);

        virtual void setMargin(glm::vec4 margin);
        glm::vec4 getMargin() const;

        /// @brief Specifies the stack order of an element
        /// @attention Is not supported by Panel
        virtual void setZIndex(int idx);
        
        /// @brief Get element z-index
        int getZIndex() const;

        virtual UINode* listenAction(const onaction& action);
        virtual UINode* listenDoubleClick(const onaction& action);
        virtual UINode* listenFocus(const onaction& action);
        virtual UINode* listenDefocus(const onaction& action);

        virtual void onFocus();
        virtual void doubleClick(int x, int y);
        virtual void click(int x, int y);
        virtual void clicked(Mousecode button) {}
        virtual void mouseMove(int x, int y) {};
        virtual void mouseRelease(int x, int y);
        virtual void scrolled(int value);

        bool isPressed() const;
        void defocus();
        bool isFocused() const; 

        /// @brief Check if element catches all user input when focused
        virtual bool isFocuskeeper() const {return false;}

        virtual void typed(unsigned int codepoint) {};
        virtual void keyPressed(Keycode key) {};

        /// @brief Check if screen position is inside of the element 
        /// @param pos screen position
        virtual bool isInside(glm::vec2 pos);

        /// @brief Get element under the cursor.
        /// @param pos cursor screen position
        /// @param self shared pointer to element
        /// @return self, sub-element or nullptr if element is not interractive
        virtual std::shared_ptr<UINode> getAt(const glm::vec2& pos);

        /// @brief Check if element is opaque for cursor
        virtual bool isInteractive() const;
        /// @brief Make the element opaque (true) or transparent (false) for cursor
        virtual void setInteractive(bool flag);

        virtual void setResizing(bool flag);
        virtual bool isResizing() const;

        virtual void setTooltip(const std::wstring& text);
        virtual const std::wstring& getTooltip() const;

        virtual void setTooltipDelay(float delay);
        virtual float getTooltipDelay() const;

        virtual void setCursor(CursorShape shape);
        virtual CursorShape getCursor() const;

        virtual glm::vec4 calcColor() const;

        /// @brief Get inner content offset. Used for scroll
        virtual glm::vec2 getContentOffset() {return glm::vec2(0.0f);};
        /// @brief Calculate screen position of the element
        virtual glm::vec2 calcPos() const;
        virtual void setPos(glm::vec2 pos);
        virtual glm::vec2 getPos() const;
        glm::vec2 getSize() const;
        virtual void setSize(glm::vec2 size);
        glm::vec2 getMinSize() const;
        virtual void setMinSize(glm::vec2 size);
        glm::vec2 getMaxSize() const;
        virtual void setMaxSize(glm::vec2 size);
        /// @brief Called in containers when new element added
        virtual void refresh() {};
        virtual void fullRefresh() {
            if (parent) {
                parent->fullRefresh();
            }
        };
        static void moveInto(
            const std::shared_ptr<UINode>& node,
            const std::shared_ptr<Container>& dest
        );

        virtual vec2supplier getPositionFunc() const;
        virtual void setPositionFunc(vec2supplier);

        virtual vec2supplier getSizeFunc() const;
        virtual void setSizeFunc(vec2supplier);

        void setId(const std::string& id);
        const std::string& getId() const;

        /// @brief Fetch pos from positionfunc if assigned
        virtual void reposition();

        virtual void setGravity(Gravity gravity);

        bool isSubnodeOf(const UINode* node);

        /// @brief collect all nodes having id
        static void getIndices(
            const std::shared_ptr<UINode>& node,
            std::unordered_map<std::string, std::shared_ptr<UINode>>& map
        );

        static std::shared_ptr<UINode> find(
            const std::shared_ptr<UINode>& node,
            const std::string& id
        );

        void setMustRefresh() {
            mustRefresh = true;
        }
    };
}
