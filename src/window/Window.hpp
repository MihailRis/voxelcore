#pragma once

#include <glm/glm.hpp>
#include <memory>

#include "graphics/core/commons.hpp"

class ImageData;
class Input;
struct DisplaySettings;

enum class WindowMode { WINDOWED, FULLSCREEN, BORDERLESS };

class Window {
public:
    static inline constexpr int FPS_UNLIMITED = 0;

    Window(glm::ivec2 size) : size(std::move(size)) {}

    virtual ~Window() = default;
    virtual void swapBuffers() noexcept = 0;

    virtual bool isMaximized() const = 0;
    virtual bool isFocused() const = 0;
    virtual bool isIconified() const = 0;

    virtual bool isShouldClose() const = 0;
    virtual void setShouldClose(bool flag) = 0;

    virtual void setCursor(CursorShape shape) = 0;
    virtual void setMode(WindowMode mode) = 0;
    virtual WindowMode getMode() const = 0;

    virtual void focus() = 0;

    virtual void setTitle(const std::string& title) = 0;
    virtual void setIcon(const ImageData* image) = 0;

    virtual void pushScissor(glm::vec4 area) = 0;
    virtual void popScissor() = 0;
    virtual void resetScissor() = 0;

    virtual void setShouldRefresh() = 0;
    virtual bool checkShouldRefresh() = 0;

    virtual double time() = 0;

    virtual void setFramerate(int framerate) = 0;

    // todo: move somewhere
    virtual std::unique_ptr<ImageData> takeScreenshot() = 0;

    [[nodiscard]] virtual bool isValid() const = 0;

    virtual const glm::ivec2& getSize() const {
        return size;
    }
    virtual void setSize(const glm::ivec2& s) {
        size = std::move(s);
    }

    static std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>>
    initialize(DisplaySettings* settings, std::string title);
protected:
    glm::ivec2 size;
    WindowMode mode = WindowMode::WINDOWED;
};

namespace display {
    void clear();
    void clearDepth();
    void setBgColor(glm::vec3 color);
    void setBgColor(glm::vec4 color);
};
