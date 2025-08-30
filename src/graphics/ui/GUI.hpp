#pragma once

#include "delegates.hpp"

#include <queue>
#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <unordered_map>

class DrawContext;
class Assets;
class Camera;
class Batch2D;
struct CursorState;
class Engine;
class Input;
class Window;

namespace devtools {
    class Editor;
}

/*
 Some info about padding and margin.
    Padding is element inner space, margin is outer
    glm::vec4 usage:
      x - left
      y - top
      z - right
      w - bottom

 Outer element
 +======================================================================+
 |            .           .                    .          .             |
 |            .padding.y  .                    .          .             |
 | padding.x  .           .                    .          .   padding.z |
 |- - - - - - + - - - - - + - - - - - - - - - -+- - - - - + - - - - - - |
 |            .           .                    .          .             |
 |            .           .margin.y            .          .             |
 |            .margin.x   .                    .  margin.z.             |
 |- - - - - - + - - - - - +====================+- - - - - + - - - - - - |
 |            .           |    Inner element   |          .             |
 |- - - - - - + - - - - - +====================+- - - - - + - - - - - - |
 |            .           .                    .          .             |
 |            .           .margin.w            .          .             |
 |            .           .                    .          .             |
 |- - - - - - + - - - - - + - - - - - - - - - -+- - - - - + - - - - - - |
 |            .           .                    .          .             |
 |            .padding.w  .                    .          .             |
 |            .           .                    .          .             |
 +======================================================================+
*/

namespace gui {
    class UINode;
    class Container;
    class Menu;

    using PageLoaderFunc = std::function<std::shared_ptr<UINode>(const std::string&)>;

    /// @brief The main UI controller
    class GUI {
        Engine& engine;
        Input& input;
        std::unique_ptr<Batch2D> batch2D;
        std::shared_ptr<Container> container;
        std::shared_ptr<UINode> hover;
        std::shared_ptr<UINode> pressed;
        std::shared_ptr<UINode> focus;
        std::shared_ptr<UINode> tooltip;
        std::unordered_map<std::string, std::shared_ptr<UINode>> storage;

        std::unique_ptr<Camera> uicamera;
        std::shared_ptr<Menu> menu;
        std::queue<runnable> postRunnables;

        PageLoaderFunc pagesLoader;

        float tooltipTimer = 0.0f;
        float doubleClickTimer = 0.0f;
        float doubleClickDelay = 0.5f;
        bool doubleClicked = false;
        bool debug = false;

        void actMouse(float delta, const CursorState& cursor);
        void actFocused();
        void updateTooltip(float delta);
        void resetTooltip();
    public:
        static constexpr int CONTEXT_MENU_ZINDEX = 999;

        GUI(Engine& engine);
        ~GUI();

        void setPageLoader(PageLoaderFunc pageLoader);
        
        PageLoaderFunc getPagesLoader();

        /// @brief Get the main menu (Menu) node
        std::shared_ptr<Menu> getMenu();

        /// @brief Get current focused node 
        /// @return focused node or nullptr
        std::shared_ptr<UINode> getFocused() const;

        /// @brief Check if all user input is caught by some element like TextBox
        bool isFocusCaught() const;

        /// @brief Main input handling and logic update method 
        /// @param delta delta time
        /// @param viewport window size
        void act(float delta, const glm::uvec2& viewport);

        /// @brief Draw all visible elements on main container 
        /// @param pctx parent graphics context
        /// @param assets active assets storage
        void draw(const DrawContext& pctx, const Assets& assets);

        void postAct();

        /// @brief Add element to the main container
        /// @param node UI element
        void add(std::shared_ptr<UINode> node);

        /// @brief Remove node from the main container
        void remove(UINode* node) noexcept;

        void remove(const std::shared_ptr<UINode>& node) noexcept {
            return remove(node.get());
        }

        /// @brief Store node in the GUI nodes dictionary 
        /// (does not add node to the main container)
        /// @param name node key
        /// @param node target node
        void store(const std::string& name, std::shared_ptr<UINode> node);

        /// @brief Get node from the GUI nodes dictionary 
        /// @param name node key
        /// @return stored node or nullptr
        std::shared_ptr<UINode> get(const std::string& name) noexcept;

        /// @brief Remove node from the GUI nodes dictionary
        /// @param name node key 
        void remove(const std::string& name) noexcept;

        /// @brief Set node as focused 
        /// @param node new focused node or nullptr to remove focus
        void setFocus(std::shared_ptr<UINode> node);

        /// @brief Get the main container
        /// @deprecated
        std::shared_ptr<Container> getContainer() const;

        void onAssetsLoad(Assets* assets);

        void postRunnable(const runnable& callback);

        void setDoubleClickDelay(float delay);
        float getDoubleClickDelay() const;

        void toggleDebug();
        const Input& getInput() const;
        Input& getInput();
        Window& getWindow();
        devtools::Editor& getEditor();
        Engine& getEngine();
    };
}
