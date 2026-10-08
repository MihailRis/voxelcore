#include "window/wayland/WaylandWindow.hpp"

#include "settings.hpp"
#include "graphics/core/Texture.hpp"
#include "util/platform.hpp"

WaylandWindow::WaylandWindow(
    WaylandInput& input, DisplaySettings* settings, int width, int height
)
    : Window({width, height}), input(input), settings(settings) {
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
}

WaylandWindow::~WaylandWindow() {
    if (state.idleInhibitor) {
        zwp_idle_inhibitor_v1_destroy(state.idleInhibitor);
    }
    if (state.lockedPointer) {
        zwp_locked_pointer_v1_destroy(state.lockedPointer);
    }
    if (state.relativePointer) {
        zwp_relative_pointer_v1_destroy(state.relativePointer);
    }
    if (state.cursorShapeDevice) {
        wp_cursor_shape_device_v1_destroy(state.cursorShapeDevice);
    }
    if (frameCallback) {
        wl_callback_destroy(frameCallback);
        frameCallback = nullptr;
    }
    if (cacheTexture) {
        glDeleteTextures(1, &cacheTexture);
    }
    if (cacheFbo) {
        glDeleteFramebuffers(1, &cacheFbo);
    }
    if (stretchProgram) {
        glDeleteProgram(stretchProgram);
    }
    if (stretchVbo) {
        glDeleteBuffers(1, &stretchVbo);
    }
    if (stretchVao) {
        glDeleteVertexArrays(1, &stretchVao);
    }
    if (state.dataDevice) {
        wl_data_device_destroy(state.dataDevice);
    }
    if (state.decoration) {
        zxdg_toplevel_decoration_v1_destroy(state.decoration);
    }
    if (state.toplevel) {
        xdg_toplevel_destroy(state.toplevel);
    }
    if (state.xdgSurface) {
        xdg_surface_destroy(state.xdgSurface);
    }
    if (eglSurface != EGL_NO_SURFACE) {
        eglDestroySurface(eglDisplay, eglSurface);
    }
    if (eglWindow) {
        wl_egl_window_destroy(eglWindow);
    }
    if (state.surface) {
        wl_surface_destroy(state.surface);
    }
    if (eglContext != EGL_NO_CONTEXT) {
        eglDestroyContext(eglDisplay, eglContext);
    }
    if (eglDisplay != EGL_NO_DISPLAY) {
        eglTerminate(eglDisplay);
    }
}

double WaylandWindow::time() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

void WaylandWindow::swapBuffers() {
    updateResizing();
    if (resizing && !presentPending && !frameDone) {
        resetScissor();
        return;
    }
    if (resizing) {
        if (renderPending) {
            cacheContent();
            renderPending = false;
        } else {
            drawStretched();
        }
    } else {
        cacheContent();
    }
    requestFrame();
    eglSwapBuffers(eglDisplay, eglSurface);
    resetScissor();
    frameDone = false;
    presentPending = false;
    const double current = now();
    if (lastPresent > 0.0) {
        const double interval = current - lastPresent;
        if (interval > 0.001 && interval < 1.0 && !resizing &&
            frameOnCallback) {
            frameInterval = std::clamp(
                frameInterval > 0.0 ? frameInterval * 0.8 + interval * 0.2
                                    : interval,
                1.0 / 240.0,
                1.0 / 30.0
            );
        }
}
    lastPresent = current;
    nextFrameTime = current + frameInterval;
    lastSwapEnd = now();
    if (framerate > 0 && !resizing) {
        auto elapsed = time() - prevSwap;
        auto frameTime = 1.0 / framerate;
        if (elapsed < frameTime) {
            platform::sleep(
                static_cast<size_t>((frameTime - elapsed) * 1000)
            );
        }
    }
    prevSwap = time();
}

void WaylandWindow::setShouldRefresh() {
    shouldRefresh = true;
}

bool WaylandWindow::checkShouldRefresh() {
    if (shouldRefresh) {
        shouldRefresh = false;
        return true;
    }
    return false;
}

bool WaylandWindow::isMaximized() const {
    return maximized;
}

bool WaylandWindow::isFocused() const {
    return activated;
}

bool WaylandWindow::isIconified() const {
    return suspended;
}

bool WaylandWindow::isFrameRequired() const {
    return !resizing || renderPending;
}

bool WaylandWindow::isShouldClose() const {
    return shouldClose;
}

void WaylandWindow::setShouldClose(bool flag) {
    shouldClose = flag;
}

void WaylandWindow::setCursor(CursorShape shape) {
    input.setCursorShape(shape);
}

void WaylandWindow::setMode(WindowMode mode) {
    Window::mode = mode;
    if (input.isCursorLocked()) {
        input.toggleCursor();
    }
    switch (mode) {
        case WindowMode::FULLSCREEN:
            setFullscreen(true);
            setIdleInhibit(true);
            break;
        case WindowMode::BORDERLESS:
            setFullscreen(false);
            setMaximized(true);
            setIdleInhibit(true);
            break;
        case WindowMode::WINDOWED:
            setFullscreen(false);
            setMaximized(false);
            setIdleInhibit(false);
            resize(settings->width.get(), settings->height.get());
            break;
    }
    wl_surface_commit(state.surface);
}

WindowMode WaylandWindow::getMode() const {
    return mode;
}

void WaylandWindow::focus() {
    if (!focusWarning) {
        focusWarning = true;
        waylandLogger.warning() << "raising windows is not allowed on Wayland";
    }
}

void WaylandWindow::setTitle(const std::string& title) {
    this->title = title;
    xdg_toplevel_set_title(state.toplevel, title.c_str());
}

void WaylandWindow::setFullscreen(bool enabled) {
    fullscreen = enabled;
    if (enabled) {
        xdg_toplevel_set_fullscreen(state.toplevel, nullptr);
    } else {
        xdg_toplevel_unset_fullscreen(state.toplevel);
    }
}

void WaylandWindow::setMaximized(bool enabled) {
    if (enabled) {
        xdg_toplevel_set_maximized(state.toplevel);
    } else {
        xdg_toplevel_unset_maximized(state.toplevel);
    }
}

void WaylandWindow::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return;
    }
    if (size.x != width || size.y != height) {
        markResizing();
    }
    presentPending = true;
    if (eglWindow) {
        wl_egl_window_resize(eglWindow, width, height, 0, 0);
    }
    glViewport(0, 0, width, height);
    size = {width, height};
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
}

void WaylandWindow::onConfigure(int32_t width, int32_t height, wl_array* states) {
    pendingWidth = width;
    pendingHeight = height;
    stateMaximized = false;
    stateSuspended = false;
    stateActivated = false;
    const auto* values = static_cast<const uint32_t*>(states->data);
    for (size_t i = 0; i < states->size / sizeof(uint32_t); i++) {
        switch (values[i]) {
            case XDG_TOPLEVEL_STATE_MAXIMIZED:
                stateMaximized = true;
                break;
            case XDG_TOPLEVEL_STATE_SUSPENDED:
                stateSuspended = true;
                break;
            case XDG_TOPLEVEL_STATE_ACTIVATED:
                stateActivated = true;
                break;
        }
    }
}

void WaylandWindow::applyConfigure() {
    if (pendingWidth > 0 && pendingHeight > 0) {
        resize(pendingWidth, pendingHeight);
    }
    maximized = stateMaximized;
    suspended = stateSuspended;
    activated = stateActivated;
    configured = true;
    setShouldRefresh();
}

void WaylandWindow::pushScissor(glm::vec4 area) {
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

void WaylandWindow::popScissor() {
    if (scissorStack.empty()) {
        waylandLogger.warning() << "extra Window::popScissor call";
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

void WaylandWindow::resetScissor() {
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
    scissorStack = std::stack<glm::vec4>();
    glDisable(GL_SCISSOR_TEST);
}

std::unique_ptr<ImageData> WaylandWindow::takeScreenshot() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GLint readBuffer = 0;
    glGetIntegerv(GL_READ_BUFFER, &readBuffer);
    glReadBuffer(GL_FRONT);
    auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, size.x, size.y, GL_RGB, GL_UNSIGNED_BYTE, data.get());
    return std::make_unique<ImageData>(
        ImageFormat::RGB888, size.x, size.y, data.release()
    );
    glReadBuffer(static_cast<GLenum>(readBuffer));
}

void WaylandWindow::setFramerate(int framerate) {
    this->framerate = framerate;
}

void WaylandWindow::initStretchRenderer() {
    static const char* vertexSource =
        "#version 330 core\n"
        "layout(location = 0) in vec2 aPos;\n"
        "out vec2 vUV;\n"
        "void main() {\n"
        "    vUV = aPos * 0.5 + 0.5;\n"
        "    gl_Position = vec4(aPos, 0.0, 1.0);\n"
        "}\n";
    static const char* fragmentSource =
        "#version 330 core\n"
        "in vec2 vUV;\n"
        "out vec4 color;\n"
        "uniform sampler2D uTexture;\n"
        "void main() {\n"
        "    color = texture(uTexture, vUV);\n"
        "}\n";
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexSource, nullptr);
    glCompileShader(vertexShader);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);
    glCompileShader(fragmentShader);
    stretchProgram = glCreateProgram();
    glAttachShader(stretchProgram, vertexShader);
    glAttachShader(stretchProgram, fragmentShader);
    glLinkProgram(stretchProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    const float vertices[] {-1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f};
    glGenVertexArrays(1, &stretchVao);
    glGenBuffers(1, &stretchVbo);
    glBindVertexArray(stretchVao);
    glBindBuffer(GL_ARRAY_BUFFER, stretchVbo);
    glBufferData(
        GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW
    );
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glBindVertexArray(0);
    glGenFramebuffers(1, &cacheFbo);
    glGenTextures(1, &cacheTexture);
}

void WaylandWindow::resizeCache(int width, int height) {
    if (cacheWidth == width && cacheHeight == height) {
        return;
    }
    cacheWidth = width;
    cacheHeight = height;
    glBindTexture(GL_TEXTURE_2D, cacheTexture);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, cacheFbo);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, cacheTexture,
        0
    );
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void WaylandWindow::cacheContent() {
    GlStateGuard state;
    if (stretchProgram == 0) {
        return;
    }
    resizeCache(size.x, size.y);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cacheFbo);
    glBlitFramebuffer(
        0, 0, size.x, size.y, 0, 0, size.x, size.y, GL_COLOR_BUFFER_BIT,
        GL_NEAREST
    );
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    lastCacheTime = now();
}

void WaylandWindow::drawStretched() {
    GlStateGuard state;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, size.x, size.y);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    if (stretchProgram == 0 || cacheWidth == 0) {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        return;
    }
    glUseProgram(stretchProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, cacheTexture);
    glUniform1i(glGetUniformLocation(stretchProgram, "uTexture"), 0);
    glBindVertexArray(stretchVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glUseProgram(0);
}

void WaylandWindow::onFrameDone() {
    frameCallback = nullptr;
    frameDone = true;
}

void WaylandWindow::requestFrame() {
    if (state.surface == nullptr || frameCallback != nullptr) {
        return;
    }
    frameCallback = wl_surface_frame(state.surface);
    wl_callback_add_listener(frameCallback, &frameListener(), this);
}

void WaylandWindow::paceFrame() {
    frameOnCallback = frameDone;
    if (lastSwapEnd > 0.0) {
        const double cost = now() - lastSwapEnd;
        if (cost > 0.0 && cost < 2.0) {
            frameCost = frameCost > 0.0 ? frameCost * 0.8 + cost * 0.2 : cost;
            cacheTimeout = std::clamp(
                frameCost * 16.0, MIN_CACHE_TIMEOUT, MAX_CACHE_TIMEOUT
            );
        }
    }
    updateResizing();
    if (resizing) {
        if (!renderPending && now() - lastResize > RESIZE_PAUSE &&
            now() - lastCacheTime > cacheTimeout) {
            renderPending = true;
        }
        dispatch_events(frameDone || presentPending ? 0 : 8);
        frameOnCallback = frameDone;
        return;
    }
    if (framerate != -1 || frameCallback == nullptr || frameDone) {
        return;
    }
    const double deadline = now() + FRAME_TIMEOUT;
    while (!frameDone && !resizing && now() < deadline) {
        dispatch_events(5);
    }
    while (!frameDone && !resizing && now() < nextFrameTime) {
        dispatch_events(5);
    }
    nextFrameTime = now() + frameInterval;
    frameOnCallback = frameDone;
}

void WaylandWindow::markResizing() {
    lastResize = now();
    if (!resizing) {
        resizing = true;
        setShouldRefresh();
    }
}

void WaylandWindow::updateResizing() {
    if (resizing && now() - lastResize > RESIZE_TIMEOUT) {
        resizing = false;
        renderPending = true;
    }
}

void WaylandWindow::setIdleInhibit(bool enabled) {
    if (enabled == (state.idleInhibitor != nullptr)) {
        return;
    }
    if (enabled && state.idleInhibitManager) {
        state.idleInhibitor = zwp_idle_inhibit_manager_v1_create_inhibitor(
            state.idleInhibitManager, state.surface
        );
    } else if (state.idleInhibitor) {
        zwp_idle_inhibitor_v1_destroy(state.idleInhibitor);
        state.idleInhibitor = nullptr;
    }
}

const wl_callback_listener& WaylandWindow::frameListener() {
    static wl_callback_listener listener = [] {
        wl_callback_listener value {};
        value.done = [] (void* data, wl_callback* callback, uint32_t) {
            wl_callback_destroy(callback);
            static_cast<WaylandWindow*>(data)->onFrameDone();
        };
        return value;
    }();
    return listener;
}

double WaylandWindow::now() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}
