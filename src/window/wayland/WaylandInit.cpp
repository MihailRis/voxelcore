#include "window/wayland/WaylandWindow.hpp"
#include "window/wayland/WaylandInput.hpp"
#include "window/wayland/WaylandListeners.hpp"

#include "settings.hpp"
#include "graphics/core/Texture.hpp"
#include <EGL/eglext.h>

std::tuple<std::unique_ptr<Window>, std::unique_ptr<Input>>
wayland_window_initialize(DisplaySettings* settings, std::string title) {
    state.display = wl_display_connect(nullptr);
    if (state.display == nullptr) {
        waylandLogger.error() << "could not connect to the Wayland display";
        return {nullptr, nullptr};
    }
    state.registry = wl_display_get_registry(state.display);
    static const wl_registry_listener registryListener = [] {
        wl_registry_listener value {};
        value.global = registry_global;
        value.global_remove = registry_global_remove;
        return value;
    }();

    wl_registry_add_listener(state.registry, &registryListener, nullptr);
    wl_display_roundtrip(state.display);
    if (state.compositor == nullptr || state.wmBase == nullptr) {
        waylandLogger.error() << "the compositor does not provide xdg-shell";
        wl_display_disconnect(state.display);
        state.display = nullptr;
        return {nullptr, nullptr};
    }

    state.surface = wl_compositor_create_surface(state.compositor);

    auto inputPtr = std::make_unique<WaylandInput>();
    input = inputPtr.get();
    auto windowPtr = std::make_unique<WaylandWindow>(
        *inputPtr, settings, settings->width.get(), settings->height.get()
    );
    window = windowPtr.get();

    if (state.dataDeviceManager && state.seat) {
        state.dataDevice = wl_data_device_manager_get_data_device(
            state.dataDeviceManager, state.seat
        );
        static const wl_data_device_listener deviceListener = [] {
        wl_data_device_listener value {};
        value.data_offer = data_device_data_offer;
        value.enter = ignore_event<>;
        value.leave = ignore_event<>;
        value.motion = ignore_event<>;
        value.drop = ignore_event<>;
        value.selection = data_device_selection;
        return value;
    }();

    wl_data_device_add_listener(state.dataDevice, &deviceListener, nullptr);
    }

    {
        state.xdgSurface = xdg_wm_base_get_xdg_surface(
            state.wmBase, state.surface
        );
        state.toplevel = xdg_surface_get_toplevel(state.xdgSurface);
        window->setTitle(title);
        xdg_toplevel_set_app_id(state.toplevel, "VoxelCore");
        if (state.decorationManager) {
            state.decoration =
                zxdg_decoration_manager_v1_get_toplevel_decoration(
                    state.decorationManager, state.toplevel
                );
            static const zxdg_toplevel_decoration_v1_listener listener = [] {
                zxdg_toplevel_decoration_v1_listener value {};
                value.configure = zxdg_decoration_configure;
                return value;
            }();
            zxdg_toplevel_decoration_v1_add_listener(
                state.decoration, &listener, nullptr
            );
            zxdg_toplevel_decoration_v1_set_mode(
                state.decoration, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE
            );
        }
        if (state.decorationManager == nullptr) {
            window->enableOwnDecorations();
        }
        static const xdg_surface_listener surfaceListener = [] {
            xdg_surface_listener value {};
            value.configure = xdg_surface_configure;
            return value;
        }();
        static const xdg_toplevel_listener toplevelListener = [] {
            xdg_toplevel_listener value {};
            value.configure = toplevel_configure;
            value.close = toplevel_close;
            value.configure_bounds = toplevel_configure_bounds;
            value.wm_capabilities = toplevel_wm_capabilities;
            return value;
        }();
        xdg_surface_add_listener(state.xdgSurface, &surfaceListener, nullptr);
        xdg_toplevel_add_listener(state.toplevel, &toplevelListener, nullptr);
        wl_surface_commit(state.surface);
    }

    while (!window->configured) {
        if (wl_display_dispatch(state.display) == -1) {
            waylandLogger.error() << "the Wayland connection was lost";
            return {nullptr, nullptr};
        }
    }

    window->eglDisplay = eglGetPlatformDisplay(
        EGL_PLATFORM_WAYLAND_KHR, state.display, nullptr
    );
    if (window->eglDisplay == EGL_NO_DISPLAY ||
        eglInitialize(window->eglDisplay, nullptr, nullptr) == EGL_FALSE) {
        waylandLogger.error() << "could not initialize EGL";
        return {nullptr, nullptr};
    }
    eglBindAPI(EGL_OPENGL_API);

    EGLint samples = settings->samples.get();
    EGLConfig config = nullptr;
    EGLint configCount = 0;
    const EGLint configAttributes[] {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 0, EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8,
        EGL_SAMPLES, samples > 0 ? samples : 0,
        EGL_NONE
    };
    eglChooseConfig(window->eglDisplay, configAttributes, &config, 1, &configCount);
    if (samples > 0 && configCount == 0) {
        waylandLogger.warning() << "multisampling is not available";
        const EGLint fallbackAttributes[] {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
            EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 0, EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8,
            EGL_NONE
        };
        eglChooseConfig(
            window->eglDisplay, fallbackAttributes, &config, 1, &configCount
        );
    }
    if (configCount == 0) {
        waylandLogger.error() << "no suitable EGL config";
        return {nullptr, nullptr};
    }

    const EGLint contextAttributes[] {
        EGL_CONTEXT_MAJOR_VERSION, 3,
        EGL_CONTEXT_MINOR_VERSION, 3,
        EGL_NONE
    };
    window->eglContext = eglCreateContext(
        window->eglDisplay, config, EGL_NO_CONTEXT, contextAttributes
    );
    window->eglWindow = wl_egl_window_create(
        state.surface, window->getSize().x, window->getSize().y
    );
    window->eglSurface = eglCreateWindowSurface(
        window->eglDisplay, config,
        reinterpret_cast<EGLNativeWindowType>(window->eglWindow), nullptr
    );
    if (window->eglContext == EGL_NO_CONTEXT ||
        window->eglSurface == EGL_NO_SURFACE ||
        eglMakeCurrent(
            window->eglDisplay, window->eglSurface, window->eglSurface,
            window->eglContext
        ) == EGL_FALSE) {
        waylandLogger.error() << "could not create the OpenGL context";
        return {nullptr, nullptr};
    }
    eglSwapInterval(window->eglDisplay, 0);

    glewExperimental = GL_TRUE;
    GLenum glewError = glewInit();
    if (glewError != GLEW_OK && glewError != GLEW_ERROR_NO_GLX_DISPLAY) {
        waylandLogger.error() << "failed to initialize GLEW:\n"
                       << glewGetErrorString(glewError);
        return {nullptr, nullptr};
    }

    window->initStretchRenderer();
    glViewport(0, 0, window->getSize().x, window->getSize().y);
    glClearColor(0.0f, 0.0f, 0.0f, 1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    GLint maxTextureSize[1] {static_cast<GLint>(Texture::MAX_RESOLUTION)};
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, maxTextureSize);
    if (maxTextureSize[0] > 0) {
        Texture::MAX_RESOLUTION = maxTextureSize[0];
        waylandLogger.info() << "max texture size is " << Texture::MAX_RESOLUTION;
    }
    waylandLogger.info() << "GL Vendor: "
                  << reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    waylandLogger.info() << "GL Renderer: "
                  << reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    waylandLogger.info() << "windowing platform: Wayland (native)";

    return {std::move(windowPtr), std::move(inputPtr)};
}

void on_display_error() {
    waylandLogger.error() << "the Wayland connection was lost";
    if (window) {
        window->setShouldClose(true);
    }
}

void WaylandInput::pollEvents(bool waitForRefresh) {
    updateClipboard();
    beginFrame();
    dispatch(waitForRefresh);
    if (window) {
        window->paceFrame();
    }
    updateRepeat();
    updateBindings();
}

void WaylandInput::refreshWindow() {
    if (window) {
        window->setShouldRefresh();
    }
}
