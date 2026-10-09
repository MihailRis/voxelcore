#pragma once

#include "window/wayland/WaylandCommon.hpp"

#include <algorithm>
#include "window/wayland/WaylandSettings.hpp"
#include "window/wayland/WaylandInput.hpp"

#include "graphics/core/ImageData.hpp"
#include "window/Window.hpp"

#include <EGL/egl.h>

#include <string>
#include <vector>

class WaylandWindow : public Window {
public:
WaylandInput& input;
DisplaySettings* settings;
EGLDisplay eglDisplay = EGL_NO_DISPLAY;
EGLContext eglContext = EGL_NO_CONTEXT;
EGLSurface eglSurface = EGL_NO_SURFACE;
wl_egl_window* eglWindow = nullptr;
int32_t pendingWidth = 0;
int32_t pendingHeight = 0;
bool stateMaximized = false;
bool stateSuspended = false;
bool stateActivated = true;
bool configured = false;
static constexpr double RESIZE_TIMEOUT = 0.4;
static constexpr double FRAME_TIMEOUT = 0.025;
static constexpr double RESIZE_PAUSE = 0.12;
static constexpr double MIN_CACHE_TIMEOUT = 0.3;
static constexpr double MAX_CACHE_TIMEOUT = 3.0;
int framerate = -1;
double prevSwap = 0.0;
double lastResize = 0.0;
bool barEnabled = false;
bool fullscreen = false;
std::string title = "VoxelCore";
int hoveredButton = -1;
bool pointerInBar = false;
SettingsWatcher settingsWatcher;
bool layoutRead = false;
void applyLayoutValue(const std::string& value);
    std::vector<DecorationButton> leftButtons;
    std::vector<DecorationButton> rightButtons;
    std::vector<DecorationButtonLayout> barButtons;
    double lastLayoutCheck = 0.0;
double lastBarPress = 0.0;
double lastBarPressX = 0.0;
double lastBarPressY = 0.0;
double barPressX = 0.0;
double barPressY = 0.0;
uint32_t barPressSerial = 0;
bool barPressed = false;
bool initialSizeApplied = false;
mutable bool frameRendered = false;
bool sizeChanged = false;
bool barDragging = false;
int barPressedButton = -1;
int initialWidth = 0;
int initialHeight = 0;
double pointerX = 0.0;
double pointerY = 0.0;
bool frameOnCallback = false;
double frameInterval = 0.0;
double nextFrameTime = 0.0;
double frameCost = 0.0;
double cacheTimeout = 0.5;
bool resizing = false;
bool renderPending = true;
bool presentPending = true;
double lastPresent = 0.0;
wl_callback* frameCallback = nullptr;
bool frameDone = false;
GLuint stretchProgram = 0;
GLuint stretchVao = 0;
GLuint stretchVbo = 0;
GLuint cacheFbo = 0;
GLuint cacheTexture = 0;
int cacheWidth = 0;
int cacheHeight = 0;
double lastCacheTime = 0.0;
double lastSwapEnd = 0.0;
bool shouldRefresh = true;
bool shouldClose = false;
bool maximized = false;
bool suspended = false;
bool activated = true;
bool focusWarning = false;
std::stack<glm::vec4> scissorStack;
glm::vec4 scissorArea {};
    WaylandWindow( WaylandInput& input, DisplaySettings* settings, int width, int height );
    ~WaylandWindow() override;
    double time() override;
    void swapBuffers() override;
    void setShouldRefresh() override;
    bool checkShouldRefresh() override;
    bool isMaximized() const override;
    bool isFocused() const override;
    bool isIconified() const override;
    bool isFrameRequired() const override;
    bool isShouldClose() const override;
    void setShouldClose(bool flag) override;
    void setCursor(CursorShape shape) override;
    void setMode(WindowMode mode) override;
    WindowMode getMode() const override;
    void focus() override;
    void setTitle(const std::string& title) override;
    const std::string& getTitle() const override;
    int getDecorationHeight() const override;
    int getDecorationHoveredButton() const override;




    static constexpr int BAR_HEIGHT = 48;
    static constexpr int BAR_BUTTON = 24;
    static constexpr int BAR_PADDING = 12;
    static constexpr int BAR_GAP = 12;
    static constexpr int BAR_EDGE = 8;
    static constexpr int SHADOW_MARGIN = 10;
    static constexpr int RESIZE_GRAB_EXTRA = 2;
    bool shadowEnabled = false;
    GLuint shadowProgram = 0;

    GLint shadowUniforms[7] {};

    int marginSize() const;

    double contentX(double x) const {
        return x - marginSize();
    }

    double contentY(double y) const {
        return y - marginSize();
    }

    glm::ivec2 surfaceSize() const;

    void updateWindowGeometry();

    void initShadowRenderer();

    void drawShadow();

    void composeFrame();

    void makeContentOpaque(int x, int y, int width, int height);


    int decorationOffset() const {
        return barVisible() ? BAR_HEIGHT : 0;
    }

    const std::vector<DecorationButtonLayout>& getDecorationButtons() const override;

    void updateButtonLayout();

    void updateBarGeometry();

    bool isIconSupported() const override {
        return false;
    }

    void setIcon(const ImageData*) override {
    }
    void setFullscreen(bool enabled);
    void setMaximized(bool enabled);
    void resize(int width, int height);
    void onConfigure(int32_t width, int32_t height, wl_array* states);
    void applyConfigure();
    void pushScissor(glm::vec4 area) override;
    void popScissor() override;
    void resetScissor() override;
    std::unique_ptr<ImageData> takeScreenshot() override;
    void setFramerate(int framerate) override;
    void initStretchRenderer();
    void resizeCache(int width, int height);
    void cacheContent();
    void drawStretched();
    void enableOwnDecorations();
    bool barVisible() const;
                            int barButtonAt(double x, double y) const;
    uint32_t barEdgeAt(double x, double y) const;
    static CursorShape edgeCursor(uint32_t edges);
    uint32_t pointerEdges() const;
    bool handleBarMotion(double x, double y);
    bool handleBarButton(uint32_t serial, int button, bool pressed);
    bool handleBarScroll() const;
    void onPointerLeave();
    void onFrameDone();
    void requestFrame();
    void paceFrame();
    void markResizing();
    void updateResizing();
    void setIdleInhibit(bool enabled);
    static const wl_callback_listener& frameListener();
    static double now();
};
