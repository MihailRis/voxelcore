#include "window/Window.hpp"
#include "window/detail/BaseInput.hpp"
#include "window/detail/WindowBackends.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <unordered_set>

#include <chrono>
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
        int height
    )
        : Window({width, height}),
          input(glfwInput),
          settings(settings),
          window(window) {
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
            glfwSetWindowAttrib(window, GLFW_DECORATED, GLFW_TRUE);
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
    GLFWwindow* window;
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
        *inputPtr, window, settings, width, height
    );
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
