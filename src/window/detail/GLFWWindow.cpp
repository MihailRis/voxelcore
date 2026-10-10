#include "window/Window.hpp"
#include "window/detail/BaseInput.hpp"
#include "window/detail/FrameLayout.hpp"
#include "window/detail/FrameOperations.hpp"
#include "window/detail/WindowBackends.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <unordered_set>

#include <chrono>
#include <cmath>
#include <stack>
#include <vector>

#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"
#include "settings.hpp"
#include "util/ObjectsKeeper.hpp"
#include "util/platform.hpp"
#include "window/input.hpp"

static debug::Logger logger("window");

static const char* get_platform_name() {
#if GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 4)
    switch (glfwGetPlatform()) {
        case GLFW_PLATFORM_WIN32: return "Win32";
        case GLFW_PLATFORM_COCOA: return "Cocoa";
        case GLFW_PLATFORM_WAYLAND: return "Wayland (GLFW)";
        case GLFW_PLATFORM_X11: return "X11";
        case GLFW_PLATFORM_NULL: return "Null";
    }
#endif
    return "unknown";
}

/// @brief Whether the engine draws the window frame itself.
///
/// Only where the client is allowed to move and resize its own window:
/// wayland forbids that, and on macOS the system frame is the expected one.
static bool use_engine_frame() {
#if defined(_WIN32)
    return true;
#elif defined(__linux__)
#if GLFW_VERSION_MAJOR > 3 || \
    (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 4)
    return glfwGetPlatform() == GLFW_PLATFORM_X11;
#else
    // GLFW 3.3 has no wayland backend at all
    return true;
#endif
#else
    return false;
#endif
}

static std::unordered_set<std::string> supported_gl_extensions;
static void window_size_callback(GLFWwindow* window, int width, int height);

static GLFWmonitor* get_window_monitor(GLFWwindow* window) {
    int x = 0, y = 0, width = 0, height = 0;
    glfwGetWindowPos(window, &x, &y);
    glfwGetWindowSize(window, &width, &height);
    x += width / 2;
    y += height / 2;
    int count = 0;
    GLFWmonitor** monitors = glfwGetMonitors(&count);
    for (int i = 0; i < count; i++) {
        int monitorX = 0, monitorY = 0;
        glfwGetMonitorPos(monitors[i], &monitorX, &monitorY);
        const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
        if (mode && x >= monitorX && x < monitorX + mode->width &&
            y >= monitorY && y < monitorY + mode->height) {
            return monitors[i];
        }
    }
    return glfwGetPrimaryMonitor();
}

static void init_gl_extensions_list() {
    GLint numExtensions = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &numExtensions);

    for (GLint i = 0; i < numExtensions; ++i) {
        const char *ext = reinterpret_cast<const char *>(glGetStringi(GL_EXTENSIONS, i));
        if (ext) {
            supported_gl_extensions.insert(ext);
        }
    }
}

[[maybe_unused]]
static bool is_gl_extension_supported(const char *extension) {
    if (!extension || !*extension) {
        return false;
    }
    return supported_gl_extensions.find(extension) != supported_gl_extensions.end();
}

#ifndef __APPLE__
static const char* gl_error_name(int error) {
    switch (error) {
        case GL_DEBUG_TYPE_ERROR: return "ERROR";
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: return "DEPRECATED_BEHAVIOR";
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: return "UNDEFINED_BEHAVIOR";
        case GL_DEBUG_TYPE_PORTABILITY: return "PORTABILITY";
        case GL_DEBUG_TYPE_PERFORMANCE: return "PERFORMANCE";
        case GL_DEBUG_TYPE_OTHER: return "OTHER";
    }
    return "UNKNOWN";
}

static const char* gl_severity_name(int severity) {
    switch (severity) {
        case GL_DEBUG_SEVERITY_LOW: return "LOW";
        case GL_DEBUG_SEVERITY_MEDIUM: return "MEDIUM";
        case GL_DEBUG_SEVERITY_HIGH: return "HIGH";
        case GL_DEBUG_SEVERITY_NOTIFICATION: return "NOTIFICATION";
    }
    return "UNKNOWN";
}

static void GLAPIENTRY gl_message_callback(
    GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar* message,
    const void* userParam
) {
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) {
        return;
    }
    if (!ENGINE_DEBUG_BUILD && severity != GL_DEBUG_SEVERITY_HIGH) {
        return;
    }
    logger.warning() << "GL:" << gl_error_name(type) << ":"
              << gl_severity_name(severity) << ": " << message;
}
#endif

static bool initialize_gl(int width, int height) {
#ifndef __APPLE__
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(gl_message_callback, nullptr);
#endif

    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLint maxTextureSize[1] {static_cast<GLint>(Texture::MAX_RESOLUTION)};
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, maxTextureSize);
    if (maxTextureSize[0] > 0) {
        Texture::MAX_RESOLUTION = maxTextureSize[0];
        logger.info() << "max texture size is " << Texture::MAX_RESOLUTION;
    }

    const GLubyte* vendor = glGetString(GL_VENDOR);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    logger.info() << "GL Vendor: " << reinterpret_cast<const char*>(vendor);
    logger.info() << "GL Renderer: " << reinterpret_cast<const char*>(renderer);
    logger.info() << "GLFW: " << glfwGetVersionString();
    return false;
}

static const char* glfw_error_name(int error) {
    switch (error) {
        case GLFW_NO_ERROR:
            return "no error";
        case GLFW_NOT_INITIALIZED:
            return "not initialized";
        case GLFW_NO_CURRENT_CONTEXT:
            return "no current context";
        case GLFW_INVALID_ENUM:
            return "invalid enum";
        case GLFW_INVALID_VALUE:
            return "invalid value";
        case GLFW_OUT_OF_MEMORY:
            return "out of memory";
        case GLFW_API_UNAVAILABLE:
            return "api unavailable";
        case GLFW_VERSION_UNAVAILABLE:
            return "version unavailable";
        case GLFW_PLATFORM_ERROR:
            return "platform error";
        case GLFW_FORMAT_UNAVAILABLE:
            return "format unavailable";
        case GLFW_NO_WINDOW_CONTEXT:
            return "no window context";
        default:
            return "unknown error";
    }
}

static void glfw_error_callback(int error, const char* description) {
    auto logline = logger.error();
    logline << "GLFW error [0x" << std::hex << error << " - "
            << glfw_error_name(error) << "]";
    if (description) {
        logline << ": " << description;
    }
}

static GLFWcursor* standard_cursors[static_cast<int>(CursorShape::LAST) + 1] = {};

/// @brief Map extra cursor shapes onto the ones GLFW knows about
static int cursor_index(CursorShape shape) {
    switch (shape) {
        case CursorShape::N_RESIZE:
        case CursorShape::S_RESIZE:
            return static_cast<int>(CursorShape::NS_RESIZE);
        case CursorShape::E_RESIZE:
        case CursorShape::W_RESIZE:
            return static_cast<int>(CursorShape::EW_RESIZE);
        case CursorShape::NE_RESIZE:
        case CursorShape::SW_RESIZE:
            return static_cast<int>(CursorShape::NESW_RESIZE);
        case CursorShape::NW_RESIZE:
        case CursorShape::SE_RESIZE:
            return static_cast<int>(CursorShape::NWSE_RESIZE);
        default:
            return static_cast<int>(shape);
    }
}

class GLFWInput : public BaseInput {
public:
    GLFWInput(GLFWwindow* window)
        : window(window) {
    }

    void pollEvents(bool waitForRefresh) override {
        beginFrame();
        if (waitForRefresh) {
            glfwWaitEventsTimeout(0.5);
        } else {
            glfwPollEvents();
        }

        updateBindings();
    }

    const char* getClipboardText() const override {
        return glfwGetClipboardString(window);
    }

    void setClipboardText(const char* text) override {
        glfwSetClipboardString(window, text);
    }

    void toggleCursor() override {
        cursorDrag = false;
        if (cursorLocked) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        } else {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            glfwSetCursor(window, nullptr);
        }
        cursorLocked = !cursorLocked;
    }
private:
    GLFWwindow* window;
};
static_assert(!std::is_abstract<GLFWInput>());

class GLFWWindow : public Window {
public:
    GLFWInput& input;
    DisplaySettings* settings;

    GLFWWindow(
        GLFWInput& glfwInput,
        GLFWwindow* window,
        DisplaySettings* settings,
        int width,
        int height,
        bool engineFrame
    )
        : Window({width, height}),
          input(glfwInput),
          settings(settings),
          window(window),
          engineFrame(engineFrame) {
        if (engineFrame) {
            frame_setup_native(window, &frame_hit_test, this);
        }
    }

    ~GLFWWindow() {
        for (int i = 0; i <= static_cast<int>(CursorShape::LAST); i++) {
            if (standard_cursors[i] != nullptr) {
                glfwDestroyCursor(standard_cursors[i]);
            }
        }
        glfwTerminate();
    }

    double time() override {
        return glfwGetTime();
    }

    void swapBuffers() override {
        glfwSwapBuffers(window);
        resetScissor();
        if (framerate > 0) {
            auto elapsedTime = time() - prevSwap;
            auto frameTime = 1.0 / framerate;
            if (elapsedTime < frameTime) {
                platform::sleep(
                    static_cast<size_t>((frameTime - elapsedTime) * 1000)
                );
            }
        }
        prevSwap = time();
    }

    void setShouldRefresh() override {
        shouldRefresh = true;
    }

    bool checkShouldRefresh() override {
        if (shouldRefresh) {
            shouldRefresh = false;
            return true;
        }
        return false;
    }

    int barHeight() const {
        if (!engineFrame || mode != WindowMode::WINDOWED) {
            return 0;
        }
        return settings->compactWindowBar.get() ? frame::BAR_HEIGHT_COMPACT
                                                : frame::BAR_HEIGHT;
    }

    int getDecorationHeight() const override {
        return barHeight();
    }

    const std::vector<DecorationButtonLayout>& getDecorationButtons() const
        override {
        buttonRects.clear();
        if (barHeight() <= 0) {
            return buttonRects;
        }
        frame::Rect rects[frame::MAX_BUTTONS];
        const int count = layoutFrameButtons(rects);
        for (int i = 0; i < count; i++) {
            buttonRects.push_back({frameButtons[i], rects[i].x, rects[i].width});
        }
        return buttonRects;
    }

    int getDecorationHoveredButton() const override {
        return hoveredButton;
    }

    bool ownsCursor() const override {
        return frameCursorActive;
    }

    /// @brief What the pointer is over, in client coordinates
    frame::Hit hitTestFrame(int x, int y) const {
        const int height = barHeight();
        if (height <= 0) {
            return frame::Hit {};
        }
        frame::Rect rects[frame::MAX_BUTTONS];
        const int count = layoutFrameButtons(rects);
        if (frame::buttonAt(
                rects,
                count,
                height,
                static_cast<float>(x),
                static_cast<float>(y)
            ) >= 0) {
            return frame::Hit {};
        }
        // the resize edges win over the bar, its top strip resizes too
        const int edges = frame::edgesAt(
            getSize().x,
            getSize().y,
            height,
            static_cast<float>(x),
            static_cast<float>(y)
        );
        if (edges != 0) {
            return frame::Hit {false, edges};
        }
        return frame::Hit {y < height, 0};
    }

    /// @brief Turns a pointer move into hovering, dragging or resizing
    void handleFrameMotion(double x, double y) {
        if (input.isCursorLocked() || barHeight() <= 0) {
            hoveredButton = -1;
            frameCursorActive = false;
            return;
        }
        if (dragging || resizeEdges != 0) {
            updateFrameDrag();
        }
        updateFrameHover(x, y);
    }

    /// @brief Turns a pointer button into a frame action
    /// @return whether the frame handled the event
    bool handleFrameButton(int button, bool pressed, double x, double y) {
        if (button != GLFW_MOUSE_BUTTON_LEFT || input.isCursorLocked() ||
            barHeight() <= 0) {
            return false;
        }
        if (!pressed) {
            if (pressedButton >= 0) {
                const int index = pressedButton;
                pressedButton = -1;
                if (index == hoveredButton) {
                    activateFrameButton(index);
                }
                return true;
            }
            if (dragging) {
                dragging = false;
                if (frameMoved) {
                    lastBarClick = 0.0;
                }
                return true;
            }
            if (resizeEdges != 0) {
                resizeEdges = 0;
                return true;
            }
            return false;
        }
        updateFrameHover(x, y);
        if (hoveredButton >= 0) {
            pressedButton = hoveredButton;
            return true;
        }
        const frame::Hit hit = hitTestFrame(
            static_cast<int>(x), static_cast<int>(y)
        );
        if (hit.caption) {
            const double now = glfwGetTime();
            if (lastBarClick > 0.0 && now - lastBarClick < DOUBLE_CLICK_TIME) {
                lastBarClick = 0.0;
                toggleMaximize();
                return true;
            }
            lastBarClick = now;
            if (frame_start_move(window)) {
                return true;
            }
            dragging = true;
            frameMoved = false;
            grabX = x;
            grabY = y;
            return true;
        }
        if (hit.edges == 0) {
            return false;
        }
        if (frame_start_resize(window, hit.edges)) {
            return true;
        }
        resizeEdges = hit.edges;
        resizeStartW = getSize().x;
        resizeStartH = getSize().y;
        glfwGetWindowPos(window, &resizeStartX, &resizeStartY);
        grabScreenX = resizeStartX + x;
        grabScreenY = resizeStartY + y;
        return true;
    }

    bool isMaximized() const override {
        return glfwGetWindowAttrib(window, GLFW_MAXIMIZED);
    }

    bool isFocused() const override {
        return glfwGetWindowAttrib(window, GLFW_FOCUSED);
    }

    bool isIconified() const override {
        return glfwGetWindowAttrib(window, GLFW_ICONIFIED);
    }

    bool isShouldClose() const override {
        return glfwWindowShouldClose(window);
    }

    void setShouldClose(bool flag) override {
        glfwSetWindowShouldClose(window, flag);
    }

    void setCursor(CursorShape shape) override {
        if (cursor == shape) {
            return;
        }
        cursor = shape;
        // nullptr cursor is valid for GLFW
        glfwSetCursor(window, standard_cursors[cursor_index(shape)]);
    }

    void setMode(WindowMode mode) override {
        Window::mode = mode;
        GLFWmonitor* monitor = get_window_monitor(window);
        const GLFWvidmode* glfwMode = glfwGetVideoMode(monitor);
    
        if (input.isCursorLocked()){
            input.toggleCursor();
        }

        if (mode == WindowMode::FULLSCREEN) {
            const int width = glfwMode->width;
            const int height = glfwMode->height;
            const int refreshRate = glfwMode->refreshRate;
            glfwGetWindowPos(window, &posX, &posY);
            glfwSetWindowMonitor(window, monitor, 0, 0, width, height, refreshRate);
        }
        else if(mode == WindowMode::BORDERLESS) {
            glfwGetWindowPos(window, &posX, &posY);
            glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_FALSE);
            glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);
            glfwSetWindowSize(window, glfwMode->width, glfwMode->height);
            glfwSetWindowPos(window, 0, 0);
        } else {
            glfwSetWindowMonitor(
                window,
                nullptr,
                posX,
                posY,
                settings->width.get(),
                settings->height.get(),
                GLFW_DONT_CARE
            );
            applyDecoration();
            glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_TRUE);
            window_size_callback(window, settings->width.get(), settings->height.get());
        }
    
        double xPos, yPos;
        glfwGetCursorPos(window, &xPos, &yPos);
        input.setCursorPosition(xPos, yPos);
    }

    WindowMode getMode() const override {
        return mode;
    }

    void focus() override {
        glfwFocusWindow(window);
    }

    std::string title;

    const char* getBackendName() const override {
        return "glfw";
    }

    const std::string& getTitle() const override {
        return title;
    }

    void setTitle(const std::string& title) override {
        this->title = title;
        glfwSetWindowTitle(window, title.c_str());
    }

    bool isIconSupported() const override {
        return true;
    }

    void setIcon(const ImageData* image) override {
        if (image == nullptr) {
            glfwSetWindowIcon(window, 0, nullptr);
            return;
        }
        GLFWimage icon {
            static_cast<int>(image->getWidth()),
            static_cast<int>(image->getHeight()),
            image->getData()};
        glfwSetWindowIcon(window, 1, &icon);
    }

    void setSize(int width, int height) {
        glViewport(0, 0, width, height);
        size = {width, height};

        if (mode == WindowMode::WINDOWED && !isMaximized()) {
            settings->width.set(width);
            settings->height.set(height);
        }
    }

                std::unique_ptr<ImageData> takeScreenshot() override {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, size.x, size.y, GL_RGB, GL_UNSIGNED_BYTE, data.get());
        return std::make_unique<ImageData>(
            ImageFormat::RGB888, size.x, size.y, data.release()
        );
    }

    void setFramerate(int framerate) override {
        if ((framerate != -1) != (this->framerate != -1)) {
            glfwSwapInterval(framerate == -1);
        }
        this->framerate = framerate;
    }
    
private:
    int layoutFrameButtons(frame::Rect* rects) const {
        return frame::layoutButtons(
            static_cast<float>(getSize().x),
            static_cast<int>(frameButtons.size()),
            settings->compactWindowBar.get(),
            rects
        );
    }

    void updateFrameHover(double x, double y) {
        const int height = barHeight();
        frame::Rect rects[frame::MAX_BUTTONS];
        const int count = layoutFrameButtons(rects);
        hoveredButton = frame::buttonAt(
            rects, count, height, static_cast<float>(x), static_cast<float>(y)
        );
        if (dragging || resizeEdges != 0) {
            return;
        }
        // inside the bar the controls win over the resize edges
        const int edges = hoveredButton < 0
            ? frame::edgesAt(
                  getSize().x,
                  getSize().y,
                  height,
                  static_cast<float>(x),
                  static_cast<float>(y)
              )
            : 0;
        if (edges != 0) {
            frameCursorActive = true;
            setCursor(edgeCursor(edges));
        } else if (y < height) {
            frameCursorActive = true;
            setCursor(CursorShape::ARROW);
        } else if (frameCursorActive) {
            // the client area below the bar belongs to the GUI, which only
            // sets a cursor when a widget is hovered
            frameCursorActive = false;
            setCursor(CursorShape::ARROW);
        }
    }

    void updateFrameDrag() {
        double x = 0.0, y = 0.0;
        glfwGetCursorPos(window, &x, &y);
        int windowX = 0, windowY = 0;
        glfwGetWindowPos(window, &windowX, &windowY);
        if (dragging) {
            const double dx = x - grabX;
            const double dy = y - grabY;
            if (std::abs(dx) > MOVE_THRESHOLD || std::abs(dy) > MOVE_THRESHOLD) {
                frameMoved = true;
            }
            glfwSetWindowPos(
                window,
                windowX + static_cast<int>(std::lround(dx)),
                windowY + static_cast<int>(std::lround(dy))
            );
            return;
        }
        const double dx = (windowX + x) - grabScreenX;
        const double dy = (windowY + y) - grabScreenY;
        int width = resizeStartW;
        int height = resizeStartH;
        int newX = resizeStartX;
        int newY = resizeStartY;
        if (resizeEdges & frame::EDGE_LEFT) {
            width = resizeStartW - static_cast<int>(std::lround(dx));
            newX = resizeStartX + static_cast<int>(std::lround(dx));
        } else if (resizeEdges & frame::EDGE_RIGHT) {
            width = resizeStartW + static_cast<int>(std::lround(dx));
        }
        if (resizeEdges & frame::EDGE_TOP) {
            height = resizeStartH - static_cast<int>(std::lround(dy));
            newY = resizeStartY + static_cast<int>(std::lround(dy));
        } else if (resizeEdges & frame::EDGE_BOTTOM) {
            height = resizeStartH + static_cast<int>(std::lround(dy));
        }
        if (width < MIN_WIDTH) {
            if (resizeEdges & frame::EDGE_LEFT) {
                newX = resizeStartX + (resizeStartW - MIN_WIDTH);
            }
            width = MIN_WIDTH;
        }
        if (height < MIN_HEIGHT) {
            if (resizeEdges & frame::EDGE_TOP) {
                newY = resizeStartY + (resizeStartH - MIN_HEIGHT);
            }
            height = MIN_HEIGHT;
        }
        glfwSetWindowSize(window, width, height);
        glfwSetWindowPos(window, newX, newY);
    }

    void activateFrameButton(int index) {
        if (index < 0 || index >= static_cast<int>(frameButtons.size())) {
            return;
        }
        switch (frameButtons[index]) {
            case DecorationButton::MINIMIZE:
                glfwIconifyWindow(window);
                break;
            case DecorationButton::MAXIMIZE:
                toggleMaximize();
                break;
            case DecorationButton::CLOSE:
                glfwSetWindowShouldClose(window, true);
                break;
        }
    }

    void toggleMaximize() {
        if (glfwGetWindowAttrib(window, GLFW_MAXIMIZED)) {
            glfwRestoreWindow(window);
        } else {
            glfwMaximizeWindow(window);
        }
        setShouldRefresh();
    }

    void applyDecoration() {
        if (engineFrame) {
            // frame_setup_native() owns the window style, GLFW would drop the
            // thick frame the native resize border and snapping need
            return;
        }
        glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_TRUE);
    }

    static frame::Hit frame_hit_test(void* userdata, int x, int y) {
        return static_cast<GLFWWindow*>(userdata)->hitTestFrame(x, y);
    }

    static CursorShape edgeCursor(int edges) {
        const bool left = edges & frame::EDGE_LEFT;
        const bool right = edges & frame::EDGE_RIGHT;
        const bool top = edges & frame::EDGE_TOP;
        const bool bottom = edges & frame::EDGE_BOTTOM;
        if (left && top) {
            return CursorShape::NW_RESIZE;
        }
        if (right && top) {
            return CursorShape::NE_RESIZE;
        }
        if (left && bottom) {
            return CursorShape::SW_RESIZE;
        }
        if (right && bottom) {
            return CursorShape::SE_RESIZE;
        }
        if (left) {
            return CursorShape::W_RESIZE;
        }
        if (right) {
            return CursorShape::E_RESIZE;
        }
        if (top) {
            return CursorShape::N_RESIZE;
        }
        if (bottom) {
            return CursorShape::S_RESIZE;
        }
        return CursorShape::ARROW;
    }

    GLFWwindow* window;
    bool engineFrame = false;
    std::vector<DecorationButton> frameButtons {
        DecorationButton::MINIMIZE,
        DecorationButton::MAXIMIZE,
        DecorationButton::CLOSE,
    };
    mutable std::vector<DecorationButtonLayout> buttonRects;
    int hoveredButton = -1;
    bool frameCursorActive = false;
    int pressedButton = -1;
    bool dragging = false;
    bool frameMoved = false;
    double grabX = 0.0;
    double grabY = 0.0;
    int resizeEdges = 0;
    int resizeStartX = 0;
    int resizeStartY = 0;
    int resizeStartW = 0;
    int resizeStartH = 0;
    double grabScreenX = 0.0;
    double grabScreenY = 0.0;
    double lastBarClick = 0.0;
    static constexpr double DOUBLE_CLICK_TIME = 0.4;
    static constexpr double MOVE_THRESHOLD = 4.0;
    static constexpr int MIN_WIDTH = 320;
    static constexpr int MIN_HEIGHT = 200;
    CursorShape cursor = CursorShape::ARROW;
    int framerate = -1;
    double prevSwap = 0.0;
    int posX = 0;
    int posY = 0;
    bool shouldRefresh = true;
};
static_assert(!std::is_abstract<GLFWWindow>());

static void mouse_button_callback(GLFWwindow* window, int button, int action, int) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    double x = 0.0, y = 0.0;
    glfwGetCursorPos(window, &x, &y);
    if (handler->handleFrameButton(button, action == GLFW_PRESS, x, y)) {
        handler->setShouldRefresh();
        return;
    }
    handler->input.onMouseCallback(button, action == GLFW_PRESS);
    handler->setShouldRefresh();
}

static void character_callback(GLFWwindow* window, unsigned int codepoint) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.codepoints.push_back(codepoint);
}

static void key_callback(
    GLFWwindow* window, int key, int /*scancode*/, int action, int /*mode*/
) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->setShouldRefresh();

    auto& input = handler->input;
    if (key == GLFW_KEY_UNKNOWN) {
        return;
    }
    if (action == GLFW_PRESS) {
        input.onKeyCallback(key, true);
        
    } else if (action == GLFW_RELEASE) {
        input.onKeyCallback(key, false);
    } else if (action == GLFW_REPEAT) {
        input.onKeyCallback(key, true);
    }
}

static void window_size_callback(GLFWwindow* window, int width, int height) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    if (width && height) {
        handler->setSize(width, height);
    }
    handler->resetScissor();
}

static void scroll_callback(GLFWwindow* window, double, double yoffset) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.scroll += yoffset;
    handler->setShouldRefresh();
}

static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.setCursorPosition(xpos, ypos);
    handler->handleFrameMotion(xpos, ypos);
    handler->setShouldRefresh();
}

static void iconify_callback(GLFWwindow* window, int iconified) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    if (handler->getMode() == WindowMode::FULLSCREEN && iconified == 0) {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        glfwSetWindowMonitor(
            window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate
        );
    }
}

static void create_standard_cursors() {
    // GLFW knows only the shapes up to NOT_ALLOWED, the rest are mapped
    // onto them by cursor_index()
    for (int i = 0; i <= static_cast<int>(CursorShape::NOT_ALLOWED); i++) {
        int cursor = GLFW_ARROW_CURSOR + i;
        // GLFW 3.3 does not support some cursors
        if (GLFW_VERSION_MAJOR <= 3 && GLFW_VERSION_MINOR <= 3 &&
            cursor > GLFW_VRESIZE_CURSOR) {
            break;
        }
        standard_cursors[i] = glfwCreateStandardCursor(cursor);
    }
}

static void refresh_callback(GLFWwindow* window) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->setShouldRefresh();
}

static void setup_callbacks(GLFWwindow* window) {
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetWindowSizeCallback(window, window_size_callback);
    glfwSetCharCallback(window, character_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetWindowIconifyCallback(window, iconify_callback);
    glfwSetWindowRefreshCallback(window, refresh_callback);
}

std::tuple<
    std::unique_ptr<Window>, 
    std::unique_ptr<Input>
> glfw_window_initialize(DisplaySettings* settings, std::string title) {
    int width = settings->width.get();
    int height = settings->height.get();

    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() == GLFW_FALSE) {
        logger.error() << "failed to initialize GLFW";
        return {nullptr, nullptr};
    }
    logger.info() << "windowing platform: " << get_platform_name();

    const bool engineFrame = use_engine_frame();
    logger.info() << "engine window frame: " << (engineFrame ? "on" : "off")
                  << (engineFrame && frame_has_native_operations()
                          ? ", window manager operations are on"
                          : "");
    if (engineFrame) {
        // the engine draws the window bar, the system frame must be gone
        glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#if GLFW_VERSION_MAJOR >= 3 && GLFW_VERSION_MINOR >= 4
    // see issue #465
    glfwWindowHint(GLFW_SCALE_FRAMEBUFFER, GL_FALSE);
#endif
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#else
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
#endif
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_SAMPLES, settings->samples.get());
    // opaque window: the UI is drawn with alpha and would otherwise
    // blend with the desktop behind the window
    glfwWindowHint(GLFW_ALPHA_BITS, 0);

    auto window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (window == nullptr) {
        logger.error() << "failed to create GLFW window";
        glfwTerminate();
        return {nullptr, nullptr};
    }
    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;

    GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK) {
        if (glewErr == GLEW_ERROR_NO_GLX_DISPLAY) {
            // see issue #240
            logger.warning()
                << "glewInit() returned GLEW_ERROR_NO_GLX_DISPLAY; ignored";
        } else {
            logger.error() << "failed to initialize GLEW:\n"
                           << glewGetErrorString(glewErr);
            glfwTerminate();
            return {nullptr, nullptr};
        }
    }

    init_gl_extensions_list();

    #ifndef __APPLE__
    if (is_gl_extension_supported("GL_KHR_debug")) {
        glEnable(GL_DEBUG_OUTPUT);
        glDebugMessageCallback(gl_message_callback, nullptr);
    }
    #endif

    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLint maxTextureSize[1] {static_cast<GLint>(Texture::MAX_RESOLUTION)};
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, maxTextureSize);
    if (maxTextureSize[0] > 0) {
        Texture::MAX_RESOLUTION = maxTextureSize[0];
        logger.info() << "max texture size is " << Texture::MAX_RESOLUTION;
    }
    setup_callbacks(window);
    
    glfwSwapInterval(1);
    create_standard_cursors();

    glm::vec2 scale;
    glfwGetMonitorContentScale(glfwGetPrimaryMonitor(), &scale.x, &scale.y);
    logger.info() << "monitor content scale: " << scale.x << "x" << scale.y;

    if (initialize_gl(width, height)) {
        glfwTerminate();
        return {nullptr, nullptr};
    }

    auto inputPtr = std::make_unique<GLFWInput>(window);
    auto windowPtr = std::make_unique<GLFWWindow>(
        *inputPtr, window, settings, width, height, engineFrame
    );
    // the engine draws the window bar from the title, and this is the only
    // place the initial one is known
    windowPtr->setTitle(title);
    glfwSetWindowUserPointer(window, windowPtr.get());
    return {std::move(windowPtr), std::move(inputPtr)};
}

void display::clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void display::clearDepth() {
    glClear(GL_DEPTH_BUFFER_BIT);
}

void display::setBgColor(glm::vec3 color) {
    glClearColor(color.r, color.g, color.b, 1.0f);
}

void display::setBgColor(glm::vec4 color) {
    glClearColor(color.r, color.g, color.b, color.a);
}
