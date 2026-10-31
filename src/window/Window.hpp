#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <stack>
#include <functional>
#include <string>
#include <vector>

#include "graphics/core/commons.hpp"
#include "typedefs.hpp"

class ImageData;
class Input;
struct DisplaySettings;

enum class DecorationButton {
    CLOSE,
    MINIMIZE,
    MAXIMIZE
};

struct DecorationButtonLayout {
    DecorationButton button;
    float x;
    float width;
};

enum class WindowMode {
    WINDOWED,
    FULLSCREEN,
    BORDERLESS
};

class Window {
public:
    static inline constexpr int FPS_UNLIMITED = 0;

    Window(glm::ivec2 size) : size(std::move(size)) {}

    virtual ~Window() = default;
    virtual void swapBuffers() = 0;

    virtual bool isMaximized() const = 0;
    virtual bool isFocused() const = 0;
    virtual bool isIconified() const = 0;

    virtual bool isShouldClose() const = 0;
    virtual void setShouldClose(bool flag) = 0;

    virtual bool isFrameRequired() const {
        return true;
    }

    virtual void setCursor(CursorShape shape) = 0;

    /// @brief Whether the engine drawn window frame owns the cursor now
    ///
    /// The frame puts the resize cursors of its edges and of the bar, and the
    /// GUI must not override them with the cursor of the hovered widget.
    virtual bool ownsCursor() const {
        return false;
    }
    virtual void setMode(WindowMode mode) = 0;
    virtual WindowMode getMode() const = 0;

    virtual void focus() = 0;

    virtual void setTitle(const std::string& title) = 0;

    virtual const std::string& getTitle() const {
        static const std::string empty;
        return empty;
    }

    /// @brief Handler the windowing system calls when it wants a repaint
    ///
    /// Windows runs a modal loop while the user drags a window border and the
    /// main loop is blocked inside it, so the only way to keep the content
    /// alive during such a resize is to draw a frame from the message loop.
    using RefreshHandler = std::function<void()>;

    void setRefreshHandler(RefreshHandler handler) {
        refreshHandler = std::move(handler);
    }

    const RefreshHandler& getRefreshHandler() const {
        return refreshHandler;
    }

    /// @brief Whether the engine draws the window frame itself here
    ///
    /// The bar height and its buttons only mean something when it does, and
    /// the window settings are only shown where that is the case.
    virtual bool hasEngineFrame() const {
        return false;
    }

    virtual const char* getBackendName() const = 0;

    virtual int getDecorationHeight() const {
        return 0;
    }

    virtual int getDecorationHoveredButton() const {
        return -1;
    }

    virtual const std::vector<DecorationButtonLayout>& getDecorationButtons()
        const {
        static const std::vector<DecorationButtonLayout> empty;
        return empty;
    }

    virtual bool isIconSupported() const = 0;

    virtual void setIcon(const ImageData* image) = 0;

    virtual void pushScissor(glm::vec4 area);
    virtual void popScissor();
    virtual void resetScissor();

    virtual void setShouldRefresh() = 0;
    virtual bool checkShouldRefresh() = 0;

    virtual double time() = 0;

    virtual void setFramerate(int framerate) = 0;

    // todo: move somewhere
    virtual std::unique_ptr<ImageData> takeScreenshot() = 0;

    const glm::ivec2& getSize() const {
        return size;
    }

    static std::tuple<
        std::unique_ptr<Window>,
        std::unique_ptr<Input>
    > initialize(DisplaySettings* settings, std::string title);
protected:
    glm::ivec2 size;
    std::stack<glm::vec4> scissorStack;
    glm::vec4 scissorArea {};
    WindowMode mode = WindowMode::WINDOWED;
    RefreshHandler refreshHandler;
};

namespace display {
    void clear();
    void clearDepth();
    void setBgColor(glm::vec3 color);
    void setBgColor(glm::vec4 color);
};
