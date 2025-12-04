#include "window/detail/SDLWindow.hpp"

#include <GL/glew.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_surface.h>
#include <SDL3/SDL_timer.h>
#include <SDL3/SDL_version.h>
#include <SDL3/SDL_video.h>

#include <memory>
#include <string>
#include <unordered_set>

#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"
#include "settings.hpp"
#include "util/platform.hpp"
#include "window/Window.hpp"
#include "window/detail/SDLInput.hpp"

static debug::Logger logger("window");

static std::unordered_set<std::string> supported_gl_extensions;
static void init_gl_extensions_list() {
    GLint numExtensions = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &numExtensions);

    for (GLint i = 0; i < numExtensions; ++i) {
        const char *ext =
            reinterpret_cast<const char *>(glGetStringi(GL_EXTENSIONS, i));
        if (ext) {
            supported_gl_extensions.insert(ext);
        }
    }
}

static bool is_gl_extension_supported(const char *extension) {
    if (!extension || !*extension) {
        return false;
    }
    return supported_gl_extensions.find(extension) !=
           supported_gl_extensions.end();
}

static const char *gl_error_name(int error) {
    switch (error) {
        case GL_DEBUG_TYPE_ERROR:
            return "ERROR";
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            return "DEPRECATED_BEHAVIOR";
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            return "UNDEFINED_BEHAVIOR";
        case GL_DEBUG_TYPE_PORTABILITY:
            return "PORTABILITY";
        case GL_DEBUG_TYPE_PERFORMANCE:
            return "PERFORMANCE";
        case GL_DEBUG_TYPE_OTHER:
            return "OTHER";
    }
    return "UNKNOWN";
}

static const char *gl_severity_name(int severity) {
    switch (severity) {
        case GL_DEBUG_SEVERITY_LOW:
            return "LOW";
        case GL_DEBUG_SEVERITY_MEDIUM:
            return "MEDIUM";
        case GL_DEBUG_SEVERITY_HIGH:
            return "HIGH";
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            return "NOTIFICATION";
    }
    return "UNKNOWN";
}

static void GLAPIENTRY gl_message_callback(
    GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar *message,
    const void *userParam
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
            return false;
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

    const GLubyte *vendor = glGetString(GL_VENDOR);
    const GLubyte *renderer = glGetString(GL_RENDERER);
    logger.info() << "GL Vendor: " << reinterpret_cast<const char *>(vendor);
    logger.info() << "GL Renderer: "
                  << reinterpret_cast<const char *>(renderer);
    logger.info() << "SDL: " << SDL_GetVersion();
    return true;
}

SDLWindow::SDLWindow(DisplaySettings *settings, std::string title) noexcept {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        logger.error() << "failed to initialize SDL: " << SDL_GetError();
        isSuccessfull = false;
        return;
    }

    SDL_WindowFlags flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;

    if (settings->windowMode.get() ==
        static_cast<int>(WindowMode::FULLSCREEN)) {
        flags |= SDL_WINDOW_FULLSCREEN;
    }

    window = SDL_CreateWindow(
        title.c_str(), settings->width.get(), settings->height.get(), flags
    );
    if (!window) {
        logger.error() << "failed to create SDL Window: " << SDL_GetError();
        isSuccessfull = false;
        return;
    }

    int width = 0;
    int height = 0;

    if (!SDL_GetWindowSize(window, &width, &height)) {
        logger.error() << "failed to get window size: " << SDL_GetError();
        isSuccessfull = false;
        return;
    }

    size = {width, height};

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#ifdef __APPLE__
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE
    );
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG
    );
#endif
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, settings->samples.get());
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    context = SDL_GL_CreateContext(window);
    if (!context) {
        logger.error() << "failed to create GL context: " << SDL_GetError();
        isSuccessfull = false;
        return;
    }

    if (!initialize_gl(width, height)) {
        logger.error() << "failed to init GL: " << SDL_GetError();
        return;
    }

    if (!SDL_GL_SetSwapInterval(1)) {
        logger.error() << "failed to set vsync: " << SDL_GetError();
        isSuccessfull = false;
        return;
    }

    renderer = SDL_CreateRenderer(window, "vc_main");

    float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());

    logger.info() << "monitor content scale: " << scale;
}
SDLWindow::~SDLWindow() {
    if (window) {
        SDL_DestroyWindow(window);
    }
    if (context) {
        if (!SDL_GL_DestroyContext(context)) {
            logger.error() << "Cant destroy gl context: " << SDL_GetError();
        }
    }
}

void SDLWindow::swapBuffers() noexcept {
    if (framerate > 0) {
        auto elapsedTime = time() - prevSwap;
        auto frameTime = 1.0 / framerate;
        if (elapsedTime < frameTime) {
            platform::sleep(
                static_cast<size_t>((frameTime - elapsedTime) * 1000)
            );
        }
        prevSwap = time();
    }

    if (!SDL_GL_SwapWindow(window)) [[unlikely]] {  // C++20 needed
        logger.error() << "Cant swap buffer: " << SDL_GetError();
    }
    resetScissor();
}
bool SDLWindow::isMaximized() const {
    return (SDL_GetWindowFlags(window) & SDL_WINDOW_MAXIMIZED) != 0;
}
bool SDLWindow::isFocused() const {
    Uint32 flags = SDL_GetWindowFlags(window);
    return (flags & SDL_WINDOW_INPUT_FOCUS) != 0 ||
           (flags & SDL_WINDOW_MOUSE_FOCUS) != 0;
}
bool SDLWindow::isIconified() const {
    return (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) != 0;
}

bool SDLWindow::isShouldClose() const {
    return toClose;
}
void SDLWindow::setShouldClose(bool flag) {
    toClose = flag;
}

void SDLWindow::setCursor(CursorShape shape) {
    // so it method called each frame, we always recreate cursor.
    // Refactor it
    if (cursor) {
        SDL_DestroyCursor(cursor);
        cursor = nullptr;
    }

    switch (shape) {
        case CursorShape::ARROW:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_DEFAULT
            );
            break;
        case CursorShape::TEXT:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_TEXT
            );
            break;
        case CursorShape::CROSSHAIR:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_CROSSHAIR
            );
            break;
        case CursorShape::POINTER:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_POINTER
            );
            break;
        case CursorShape::EW_RESIZE:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_EW_RESIZE
            );
            break;
        case CursorShape::NS_RESIZE:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_NS_RESIZE
            );
            break;
        case CursorShape::NWSE_RESIZE:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_NWSE_RESIZE
            );
            break;
        case CursorShape::NESW_RESIZE:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_NESW_RESIZE
            );
            break;
        case CursorShape::ALL_RESIZE:
            // ??
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_NWSE_RESIZE
            );
            break;
        case CursorShape::NOT_ALLOWED:
            cursor = SDL_CreateSystemCursor(
                SDL_SystemCursor::SDL_SYSTEM_CURSOR_NOT_ALLOWED
            );
            break;
    }
    if (cursor) {
        SDL_SetCursor(cursor);
    }
}
void SDLWindow::setMode(WindowMode mode) {
    // WindowMode::WINDOWED by default have the next options as disbled
    if (!SDL_SetWindowFullscreen(window, mode == WindowMode::FULLSCREEN)) {
        logger.error() << "Cant toggle fullscreen window: " << SDL_GetError();
    }
    if (!SDL_SetWindowBordered(window, mode == WindowMode::BORDERLESS)) {
        logger.error() << "Cant toggle bordered window: " << SDL_GetError();
    }
    this->mode = mode;
}

WindowMode SDLWindow::getMode() const {
    return mode;
}

void SDLWindow::setIcon(const ImageData *image) {
    if (image == nullptr) {
        logger.error() << "Image is nullptr";
        return;
    }

    SDL_Surface *iconSurface = SDL_CreateSurface(
        image->getWidth(), image->getHeight(), SDL_PIXELFORMAT_RGBA32
    );

    if (!iconSurface) {
        logger.error() << "Failed to create surface for app icon: "
                       << SDL_GetError();
        return;
    }

    memcpy(
        iconSurface->pixels,
        image->getData(),
        image->getWidth() * image->getHeight() * 4
    );

    if (!SDL_SetWindowIcon(window, iconSurface)) {
        logger.error() << "Failed to set icon: " << SDL_GetError();
    }

    SDL_DestroySurface(iconSurface);
}

void SDLWindow::pushScissor(glm::vec4 area) {
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

void SDLWindow::resetScissor() {
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
    scissorStack = std::stack<glm::vec4>();
    glDisable(GL_SCISSOR_TEST);
}

void SDLWindow::popScissor() {
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

double SDLWindow::time() {
    return static_cast<double>(SDL_GetTicksNS()) / 1'000'000'000;
}

void SDLWindow::setFramerate(int framerate) {
    if (!SDL_GL_SetSwapInterval(framerate == -1)) {
        logger.error() << "Failed to set framerate: " << SDL_GetError();
    }
    this->framerate = framerate;
}

// todo: move somewhere
// Possible do with SDL_RenderReadPixels
std::unique_ptr<ImageData> SDLWindow::takeScreenshot() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, size.x, size.y, GL_RGB, GL_UNSIGNED_BYTE, data.get());
    return std::make_unique<ImageData>(
        ImageFormat::rgb888, size.x, size.y, data.release()
    );
}

[[nodiscard]] bool SDLWindow::isValid() const {
    return isSuccessfull;
}
[[nodiscard]] SDL_Window *SDLWindow::getSdlWindow() const {
    return window;
}

std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>> Window::initialize(
    DisplaySettings *settings, std::string title
) {
    auto window = std::make_unique<SDLWindow>(settings, title);
    if (!window->isValid()) {
        return {nullptr, nullptr};
    }
    auto input = std::make_unique<SDLInput>(*window);

    return {std::move(window), std::move(input)};
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
