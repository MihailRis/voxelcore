#include "window/wayland/WaylandWindow.hpp"

#include "settings.hpp"
#include "graphics/core/Texture.hpp"
#include "util/platform.hpp"

WaylandWindow::WaylandWindow(
    WaylandInput& input, DisplaySettings* settings, int width, int height
)
    : Window({width, height}),
      input(input),
      settings(settings),
      initialWidth(width),
      initialHeight(height) {
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
}

WaylandWindow::~WaylandWindow() {
    if (window == this) {
        window = nullptr;
    }
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
    if (shadowProgram) {
        glDeleteProgram(shadowProgram);
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
    if (state.pointer) {
        wl_pointer_destroy(state.pointer);
        state.pointer = nullptr;
    }
    if (state.keyboard) {
        wl_keyboard_destroy(state.keyboard);
        state.keyboard = nullptr;
    }
    if (state.seat) {
        wl_seat_destroy(state.seat);
        state.seat = nullptr;
    }
    if (state.xkbState) {
        xkb_state_unref(state.xkbState);
        state.xkbState = nullptr;
    }
    if (state.xkbKeymap) {
        xkb_keymap_unref(state.xkbKeymap);
        state.xkbKeymap = nullptr;
    }
}

double WaylandWindow::time() {
    return now();
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
        } else if (marginSize() == 0) {
            drawStretched();
        }
    } else if (frameRendered) {
        cacheContent();
    }
    frameRendered = false;
    composeFrame();
    requestFrame();
    eglSwapBuffers(eglDisplay, eglSurface);
    if (sizeChanged) {
        sizeChanged = false;
        composeFrame();
        eglSwapBuffers(eglDisplay, eglSurface);
    }
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
    const bool required = !resizing || renderPending;
    if (required) {
        frameRendered = true;
    }
    return required;
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
            if (maximized) {
                setMaximized(false);
            }
            setIdleInhibit(false);
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

int WaylandWindow::marginSize() const {
    if (!barEnabled || !shadowEnabled) {
        return 0;
    }
    if (fullscreen || maximized || Window::mode != WindowMode::WINDOWED) {
        return 0;
    }
    return SHADOW_MARGIN;
}

glm::ivec2 WaylandWindow::surfaceSize() const {
    const int margin = marginSize() * 2;
    return {size.x + margin, size.y + margin};
}

void WaylandWindow::updateWindowGeometry() {
    if (state.xdgSurface == nullptr) {
        return;
    }
    const int margin = marginSize();
    xdg_surface_set_window_geometry(
        state.xdgSurface, margin, margin, size.x, size.y
    );
}

void WaylandWindow::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return;
    }
    if (size.x != width || size.y != height) {
        markResizing();
        sizeChanged = true;
    }
    presentPending = true;
    size = {width, height};
    if (eglWindow) {
        const auto surface = surfaceSize();
        wl_egl_window_resize(eglWindow, surface.x, surface.y, 0, 0);
    }
    glViewport(0, 0, width, height);
    updateWindowGeometry();
    if (Window::mode == WindowMode::WINDOWED && !maximized) {
        settings->width.set(width);
        settings->height.set(height);
    }
    updateBarGeometry();
    scissorArea = glm::vec4(0.0f, 0.0f, size.x, size.y);
}

void WaylandWindow::onConfigure(
    int32_t width, int32_t height, wl_array* states
) {
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
    maximized = stateMaximized;
    suspended = stateSuspended;
    if (pendingWidth > 0 && pendingHeight > 0) {
        if (!initialSizeApplied && Window::mode == WindowMode::WINDOWED &&
            !maximized && initialWidth > 0 && initialHeight > 0) {
            pendingWidth = initialWidth;
            pendingHeight = initialHeight;
        }
        initialSizeApplied = true;
        resize(pendingWidth, pendingHeight);
    }
    if (stateActivated && !activated) {
        lastLayoutCheck = 0.0;
    }
    activated = stateActivated;
    configured = true;
    setShouldRefresh();
}

std::unique_ptr<ImageData> WaylandWindow::takeScreenshot() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GLint readBuffer = 0;
    glGetIntegerv(GL_READ_BUFFER, &readBuffer);
    glReadBuffer(GL_FRONT);
    const int margin = marginSize();
    auto data = std::make_unique<ubyte[]>(size.x * size.y * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(
        margin,
        margin,
        size.x,
        size.y,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        data.get()
    );
    glReadBuffer(static_cast<GLenum>(readBuffer));
    return std::make_unique<ImageData>(
        ImageFormat::RGB888, size.x, size.y, data.release()
    );
}

void WaylandWindow::setFramerate(int framerate) {
    this->framerate = framerate;
}

void WaylandWindow::initStretchRenderer() {
    static const char* fragmentSource =
        "#version 330 core\n"
        "in vec2 vUV;\n"
        "out vec4 color;\n"
        "uniform sampler2D uTexture;\n"
        "void main() {\n"
        "    color = texture(uTexture, vUV);\n"
        "}\n";
    stretchProgram = compile_program(WAYLAND_VERTEX_SHADER, fragmentSource);
    GLint stretchLinked = 0;
    glGetProgramiv(stretchProgram, GL_LINK_STATUS, &stretchLinked);
    if (stretchProgram == 0 || stretchLinked == 0) {
        waylandLogger.error() << "failed to build the stretch shader";
        stretchProgram = 0;
        return;
    }
    stretchTextureUniform = glGetUniformLocation(stretchProgram, "uTexture");

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

void WaylandWindow::initShadowRenderer() {
    if (!shadowEnabled) {
        return;
    }
    static const char* fragmentSource =
        "#version 330 core\n"
        "in vec2 vUV;\n"
        "out vec4 color;\n"
        "uniform vec2 uSurface;\n"
        "uniform vec4 uRect;\n"
        "uniform float uBlur;\n"
        "uniform float uStrength;\n"
        "uniform float uOffset;\n"
        "uniform float uSpread;\n"
        "uniform float uBorder;\n"
        "uniform float uRadius;\n"
        "float erfcApprox(float x) {\n"
        "    float t = 1.0 / (1.0 + 0.5 * abs(x));\n"
        "    float tau = t * exp(-x * x - 1.26551223 +\n"
        "        t * (1.00002368 +\n"
        "        t * (0.37409196 +\n"
        "        t * (0.09678418 +\n"
        "        t * (-0.18628806 +\n"
        "        t * (0.27886807 +\n"
        "        t * (-1.13520398 +\n"
        "        t * (1.48851587 +\n"
        "        t * (-0.82215223 +\n"
        "        t * 0.17087277)))))))));\n"
        "    return x >= 0.0 ? tau : 2.0 - tau;\n"
        "}\n"
        "float boxDistance(vec2 p, vec2 center, vec2 halfSize, float radius)"
        " {\n"
        "    vec2 q = abs(p - center) - (halfSize - vec2(radius));\n"
        "    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;\n"
        "}\n"
        "void main() {\n"
        "    vec2 p = vUV * uSurface;\n"
        "    vec2 halfSize = uRect.zw * 0.5;\n"
        "    vec2 center = uRect.xy + halfSize;\n"
        "    float shadow = boxDistance(\n"
        "        p, center + vec2(0.0, -uOffset), halfSize + uSpread,\n"
        "        uRadius + uSpread\n"
        "    );\n"
        "    float sigma = max(uBlur * 0.5, 0.001);\n"
        "    float alpha = uStrength * 0.5 * "
        "erfcApprox(shadow / (sigma * 1.41421356));\n"
        "    float side = boxDistance(p, center, halfSize, uRadius);\n"
        "    float border = side > 0.0 && side <= 1.0 ? uBorder : 0.0;\n"
        "    color = vec4(0.0, 0.0, 0.0, max(alpha, border));\n"
        "}\n";
    shadowProgram = compile_program(WAYLAND_VERTEX_SHADER, fragmentSource);
    GLint linked = 0;
    glGetProgramiv(shadowProgram, GL_LINK_STATUS, &linked);
    if (shadowProgram == 0 || linked == 0) {
        char log[1024] = {};
        glGetProgramInfoLog(shadowProgram, sizeof(log), nullptr, log);
        for (char* c = log; *c != '\0'; c++) {
            if (*c == '\n') {
                *c = ' ';
            }
        }
        waylandLogger.error() << "failed to build the shadow shader: " << log;
        shadowEnabled = false;
        resize(size.x, size.y);
        return;
    }
    const char* names[] {
        "uSurface", "uRect", "uBlur", "uStrength", "uOffset", "uSpread",
        "uBorder", "uRadius", nullptr
    };
    for (int i = 0; names[i] != nullptr; i++) {
        shadowUniforms[i] = glGetUniformLocation(shadowProgram, names[i]);
    }

}

namespace {
    constexpr float SHADOW_BLUR = 7.0f;
    constexpr float SHADOW_SPREAD = 1.0f;
    constexpr float SHADOW_STRENGTH = 0.5f;
    constexpr float SHADOW_STRENGTH_BACKDROP = 0.3f;
    constexpr float SHADOW_BORDER = 0.1f;
    constexpr float SHADOW_BORDER_BACKDROP = 0.05f;
}

void WaylandWindow::drawShadow() {
    const int margin = marginSize();
    if (margin <= 0 || shadowProgram == 0) {
        return;
    }
    const auto surface = surfaceSize();
    glViewport(0, 0, surface.x, surface.y);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    glUseProgram(shadowProgram);
    glUniform2f(
        shadowUniforms[0],
        static_cast<float>(surface.x),
        static_cast<float>(surface.y)
    );
    glUniform4f(
        shadowUniforms[1],
        static_cast<float>(margin),
        static_cast<float>(margin),
        static_cast<float>(size.x),
        static_cast<float>(size.y)
    );
    glUniform1f(shadowUniforms[2], SHADOW_BLUR);
    glUniform1f(
        shadowUniforms[3],
        activated ? SHADOW_STRENGTH : SHADOW_STRENGTH_BACKDROP
    );
    glUniform1f(shadowUniforms[4], 0.0f);
    glUniform1f(shadowUniforms[5], SHADOW_SPREAD);
    glUniform1f(
        shadowUniforms[6],
        activated ? SHADOW_BORDER : SHADOW_BORDER_BACKDROP
    );
    glUniform1f(shadowUniforms[7], 0.0f);
    glBindVertexArray(stretchVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glUseProgram(0);
}

void WaylandWindow::makeContentOpaque(int x, int y, int width, int height) {
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
    glEnable(GL_SCISSOR_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glScissor(x, y, width, height);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void WaylandWindow::composeFrame() {
    const int margin = marginSize();
    GlStateGuard state;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (margin <= 0) {
        makeContentOpaque(0, 0, size.x, size.y);
        return;
    }
    const auto surface = surfaceSize();
    glViewport(0, 0, surface.x, surface.y);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_BLEND);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    drawShadow();
    if (stretchProgram == 0 || cacheWidth == 0) {
        glEnable(GL_SCISSOR_TEST);
        glScissor(margin, margin, size.x, size.y);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        return;
    }
    glViewport(margin, margin, size.x, size.y);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    glUseProgram(stretchProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, cacheTexture);
    glUniform1i(stretchTextureUniform, 0);
    glBindVertexArray(stretchVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glUseProgram(0);
    makeContentOpaque(margin, margin, size.x, size.y);
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

/// @brief Copies the frame the engine just drew into a texture.
/// Runs every frame: composeFrame() rebuilds the window from this texture,
/// which keeps the engine itself unaware of the shadow margins.
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
    glUniform1i(stretchTextureUniform, 0);
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
    updateButtonLayout();
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
    return waylandNow();
}
