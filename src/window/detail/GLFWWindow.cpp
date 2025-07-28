#include "window/Window.hpp"

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

static std::unordered_set<std::string> supported_gl_extensions;

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

static bool is_gl_extension_supported(const char *extension) {
    if (!extension || !*extension) {
        return false;
    }
    return supported_gl_extensions.find(extension) != supported_gl_extensions.end();
}

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

static bool initialize_gl(int width, int height) {
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
            return true;
        }
    }

#ifndef __APPLE__
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(gl_message_callback, 0);
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

inline constexpr short KEYS_BUFFER_SIZE = 1036;
inline constexpr short MOUSE_KEYS_OFFSET = 1024;

static GLFWcursor* standard_cursors[static_cast<int>(CursorShape::LAST) + 1] = {};

class GLFWInput : public Input {
public:
    int scroll = 0;
    uint currentFrame = 0;
    uint frames[KEYS_BUFFER_SIZE] {};
    std::vector<uint> codepoints;
    std::vector<Keycode> pressedKeys;
    Bindings bindings;
    bool keys[KEYS_BUFFER_SIZE] {};
    std::unordered_map<Keycode, util::HandlersList<>> keyCallbacks;

    GLFWInput(GLFWwindow* window)
        : window(window) {
    }

    void pollEvents() override {
        delta.x = 0.0f;
        delta.y = 0.0f;
        scroll = 0;
        currentFrame++;
        codepoints.clear();
        pressedKeys.clear();
        glfwPollEvents();

        for (auto& [_, binding] : bindings.getAll()) {
            if (!binding.enabled) {
                binding.state = false;
                continue;
            }
            binding.justChanged = false;
    
            bool newstate = false;
            switch (binding.type) {
                case InputType::KEYBOARD:
                    newstate = pressed(static_cast<Keycode>(binding.code));
                    break;
                case InputType::MOUSE:
                    newstate = clicked(static_cast<Mousecode>(binding.code));
                    break;
            }
    
            if (newstate) {
                if (!binding.state) {
                    binding.state = true;
                    binding.justChanged = true;
                    binding.onactived.notify();
                }
            } else {
                if (binding.state) {
                    binding.state = false;
                    binding.justChanged = true;
                }
            }
        }
    }

    void onKeyCallback(int key, bool pressed) {
        bool prevPressed = keys[key];
        keys[key] = pressed;
        frames[key] = currentFrame;
        if (pressed && !prevPressed) {
            const auto& callbacks = keyCallbacks.find(static_cast<Keycode>(key));
            if (callbacks != keyCallbacks.end()) {
                callbacks->second.notify();
            }
        }
        if (pressed && key < MOUSE_KEYS_OFFSET) {
            pressedKeys.push_back(static_cast<Keycode>(key));
        }
    }

    void onMouseCallback(int button, bool pressed) {
        int key = button + MOUSE_KEYS_OFFSET;
        onKeyCallback(key, pressed);
    }

    const char* getClipboardText() const override {
        return glfwGetClipboardString(window);
    }

    void setClipboardText(const char* text) override {
        glfwSetClipboardString(window, text);
    }

    int getScroll() override {
        return scroll;
    }

    bool pressed(Keycode key) const override {
        int keycode = static_cast<int>(key);
        if (keycode < 0 || keycode >= KEYS_BUFFER_SIZE) {
            return false;
        }
        return keys[keycode];
    }
    bool jpressed(Keycode keycode) const override {
        return pressed(keycode) &&
               frames[static_cast<int>(keycode)] == currentFrame;
    }

    bool clicked(Mousecode code) const override {
        return pressed(
            static_cast<Keycode>(MOUSE_KEYS_OFFSET + static_cast<int>(code))
        );
    }
    bool jclicked(Mousecode code) const override {
        return clicked(code) &&
               frames[static_cast<int>(code) + MOUSE_KEYS_OFFSET] ==
                   currentFrame;
    }

    CursorState getCursor() const override {
        return {isCursorLocked(), cursor, delta};
    }

    bool isCursorLocked() const override {
        return cursorLocked;
    }

    void toggleCursor() override {
        cursorDrag = false;
        if (cursorLocked) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        } else {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
        cursorLocked = !cursorLocked;
    }

    void setCursorPosition(double xpos, double ypos) {
        if (cursorDrag) {
            delta.x += xpos - cursor.x;
            delta.y += ypos - cursor.y;
        } else {
            cursorDrag = true;
        }
        cursor.x = xpos;
        cursor.y = ypos;
    }

    Bindings& getBindings() override {
        return bindings;
    }

    const Bindings& getBindings() const override {
        return bindings;
    }

    ObserverHandler addKeyCallback(Keycode key, KeyCallback callback) override {
        return keyCallbacks[key].add(std::move(callback));
    }

    const std::vector<Keycode>& getPressedKeys() const override {
        return pressedKeys;
    }

    const std::vector<uint>& getCodepoints() const override {
        return codepoints;
    }
private:
    GLFWwindow* window;
    bool cursorLocked = false;
    bool cursorDrag = false;
    glm::vec2 delta;
    glm::vec2 cursor;
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
            glfwDestroyCursor(standard_cursors[i]);
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
        // NULL cursor is valid for GLFW
        glfwSetCursor(window, standard_cursors[static_cast<int>(shape)]);
    }

    void toggleFullscreen() override {
        fullscreen = !fullscreen;
    
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    
        if (input.isCursorLocked()){
            input.toggleCursor();
        }
    
        if (fullscreen) {
            glfwGetWindowPos(window, &posX, &posY);
            glfwSetWindowMonitor(
                window, monitor, 0, 0, mode->width, mode->height, GLFW_DONT_CARE
            );
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
        }
    
        double xPos, yPos;
        glfwGetCursorPos(window, &xPos, &yPos);
        input.setCursorPosition(xPos, yPos);
    }

    bool isFullscreen() const override {
        return fullscreen;
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

        if (!isFullscreen() && !isMaximized()) {
            settings->width.set(width);
            settings->height.set(height);
        }
    }

    void pushScissor(glm::vec4 area) override {
        if (scissorStack.empty()) {
            glEnable(GL_SCISSOR_TEST);
        }
        scissorStack.push(scissorArea);
    
        area.z += glm::ceil(area.x);
        area.w += glm::ceil(area.y);
    
        area.x = glm::max(area.x, scissorArea.x);
        area.y = glm::max(area.y, scissorArea.y);
    
        area.z = glm::min(area.z, scissorArea.z);
        area.w = glm::min(area.w, scissorArea.w);
    
        if (area.z < 0.0f || area.w < 0.0f) {
            glScissor(0, 0, 0, 0);
        } else {
            glScissor(
                area.x,
                size.y - area.w,
                std::max(0, static_cast<int>(glm::ceil(area.z - area.x))),
                std::max(0, static_cast<int>(glm::ceil(area.w - area.y)))
            );
        }
        scissorArea = area;
    }

    void resetScissor() override {
        scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
        scissorStack = std::stack<glm::vec4>();
        glDisable(GL_SCISSOR_TEST);
    }

    void popScissor() override {
        if (scissorStack.empty()) {
            logger.warning() << "extra Window::popScissor call";
            return;
        }
        glm::vec4 area = scissorStack.top();
        scissorStack.pop();
        if (area.z < 0.0f || area.w < 0.0f) {
            glScissor(0, 0, 0, 0);
        } else {
            glScissor(
                area.x,
                size.y - area.w,
                std::max(0, static_cast<int>(area.z - area.x)),
                std::max(0, static_cast<int>(area.w - area.y))
            );
        }
        if (scissorStack.empty()) {
            glDisable(GL_SCISSOR_TEST);
        }
        scissorArea = area;
    }

    std::unique_ptr<ImageData> takeScreenshot() override {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, size.x, size.y, GL_RGB, GL_UNSIGNED_BYTE, data.get());
        return std::make_unique<ImageData>(
            ImageFormat::rgb888, size.x, size.y, data.release()
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
    bool fullscreen = false;
    int framerate = -1;
    std::stack<glm::vec4> scissorStack;
    glm::vec4 scissorArea;
    double prevSwap = 0.0;
    int posX = 0;
    int posY = 0;
};
static_assert(!std::is_abstract<GLFWWindow>());

static void mouse_button_callback(GLFWwindow* window, int button, int action, int) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.onMouseCallback(button, action == GLFW_PRESS);
}

static void character_callback(GLFWwindow* window, unsigned int codepoint) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.codepoints.push_back(codepoint);
}

static void key_callback(
    GLFWwindow* window, int key, int /*scancode*/, int action, int /*mode*/
) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
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
}

static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    auto handler = static_cast<GLFWWindow*>(glfwGetWindowUserPointer(window));
    handler->input.setCursorPosition(xpos, ypos);
}

static void create_standard_cursors() {
    for (int i = 0; i <= static_cast<int>(CursorShape::LAST); i++) {
        int cursor = GLFW_ARROW_CURSOR + i;
        // GLFW 3.3 does not support some cursors
        if (GLFW_VERSION_MAJOR <= 3 && GLFW_VERSION_MINOR <= 3 &&
            cursor > GLFW_VRESIZE_CURSOR) {
            break;
        }
        standard_cursors[i] = glfwCreateStandardCursor(cursor);
    }
}

static void setup_callbacks(GLFWwindow* window) {
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetWindowSizeCallback(window, window_size_callback);
    glfwSetCharCallback(window, character_callback);
    glfwSetScrollCallback(window, scroll_callback);
}

std::tuple<
    std::unique_ptr<Window>, 
    std::unique_ptr<Input>
> Window::initialize(DisplaySettings* settings, std::string title) {
    int width = settings->width.get();
    int height = settings->height.get();

    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() == GLFW_FALSE) {
        logger.error() << "failed to initialize GLFW";
        return {nullptr, nullptr};
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
    input_util::initialize();
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
