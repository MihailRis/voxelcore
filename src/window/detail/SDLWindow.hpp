#include <SDL3/SDL_render.h>
#include <SDL3/SDL_video.h>

#include <stack>

#include "window/Window.hpp"

class SDLWindow final : public Window {
public:
    SDLWindow(DisplaySettings *settings, std::string title) noexcept;
    ~SDLWindow();
    SDLWindow(const SDLWindow &) = delete;
    SDLWindow(SDLWindow &&) = default;
    SDLWindow &operator=(const SDLWindow &) = delete;
    SDLWindow &operator=(SDLWindow &&) = default;

    void swapBuffers() noexcept override;

    bool isMaximized() const override;
    bool isFocused() const override;
    bool isIconified() const override;

    bool isShouldClose() const override;
    void setShouldClose(bool flag) override;

    void setCursor(CursorShape shape) override;
    void setMode(WindowMode mode) override;
    WindowMode getMode() const override;

    void setIcon(const ImageData *image) override;

    void pushScissor(glm::vec4 area) override;
    void popScissor() override;
    void resetScissor() override;

    double time() override;

    void setFramerate(int framerate) override;

    // todo: move somewhere
    std::unique_ptr<ImageData> takeScreenshot() override;

    [[nodiscard]] bool isValid() const override;
    [[nodiscard]] SDL_Window *getSdlWindow() const;
private:
    bool isSuccessfull = true;
    bool toClose = false;
    bool fullscreen = false;

    double framerate = -1;
    double prevSwap = 0.0;

    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    SDL_Cursor *cursor = nullptr;
    SDL_GLContext context = nullptr;

    std::stack<glm::vec4> scissorStack;
    glm::vec4 scissorArea {};
};