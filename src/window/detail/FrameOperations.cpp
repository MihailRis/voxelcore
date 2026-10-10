#include "window/detail/FrameOperations.hpp"

#if defined(VOXEL_X11_FRAME)
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <dlfcn.h>

#include <GLFW/glfw3.h>

namespace {
    /// @brief X11 accessors of GLFW, resolved when they are really there
    ///
    /// A wayland-only build of GLFW does not export them, and then the frame
    /// moves the window itself instead of asking the window manager.
    struct X11Api {
        Display* (*display)() = nullptr;
        Window (*window)(GLFWwindow*) = nullptr;

        X11Api() {
            display = reinterpret_cast<Display* (*)()>(
                dlsym(RTLD_DEFAULT, "glfwGetX11Display")
            );
            window = reinterpret_cast<Window (*)(GLFWwindow*)>(
                dlsym(RTLD_DEFAULT, "glfwGetX11Window")
            );
            if (display == nullptr || window == nullptr) {
                display = nullptr;
                window = nullptr;
            }
        }

        bool available() const {
            return display != nullptr && window != nullptr;
        }
    };

    X11Api& x11_api() {
        static X11Api api;
        return api;
    }

    /// @brief Whether the window manager advertises _NET_WM_MOVERESIZE
    ///
    /// Without it the request would be silently dropped, and the frame has to
    /// move and resize the window itself. The answer is cached.
    bool x11_supports_moveresize(Display* display) {
        static int cached = -1;
        if (cached >= 0) {
            return cached == 1;
        }
        cached = 0;
        const Atom supported = XInternAtom(display, "_NET_SUPPORTED", False);
        const Atom moveresize =
            XInternAtom(display, "_NET_WM_MOVERESIZE", False);
        Atom type = 0;
        int format = 0;
        unsigned long count = 0, after = 0;
        unsigned char* data = nullptr;
        if (XGetWindowProperty(
                display,
                DefaultRootWindow(display),
                supported,
                0,
                8192,
                False,
                XA_ATOM,
                &type,
                &format,
                &count,
                &after,
                &data
            ) == Success &&
            data != nullptr) {
            const Atom* atoms = reinterpret_cast<const Atom*>(data);
            for (unsigned long i = 0; i < count; i++) {
                if (atoms[i] == moveresize) {
                    cached = 1;
                    break;
                }
            }
            XFree(data);
        }
        return cached == 1;
    }

    /// @brief Edge mask to the _NET_WM_MOVERESIZE direction
    int x11_direction(int edges) {
        switch (edges) {
            case frame::EDGE_TOP | frame::EDGE_LEFT:
                return 0;
            case frame::EDGE_TOP:
                return 1;
            case frame::EDGE_TOP | frame::EDGE_RIGHT:
                return 2;
            case frame::EDGE_RIGHT:
                return 3;
            case frame::EDGE_BOTTOM | frame::EDGE_RIGHT:
                return 4;
            case frame::EDGE_BOTTOM:
                return 5;
            case frame::EDGE_BOTTOM | frame::EDGE_LEFT:
                return 6;
            default:
                return 7;
        }
    }

    constexpr int MOVERESIZE_MOVE = 8;
    constexpr int SOURCE_APPLICATION = 1;

    bool x11_request(GLFWwindow* glfwWindow, int direction) {
        X11Api& api = x11_api();
        if (!api.available()) {
            return false;
        }
        Display* display = api.display();
        const Window window = api.window(glfwWindow);
        if (display == nullptr || window == 0 ||
            !x11_supports_moveresize(display)) {
            return false;
        }
        const Window root = DefaultRootWindow(display);
        int rootX = 0, rootY = 0, windowX = 0, windowY = 0;
        Window child = 0, rootReturn = 0;
        unsigned int mask = 0;
        XQueryPointer(
            display,
            root,
            &rootReturn,
            &child,
            &rootX,
            &rootY,
            &windowX,
            &windowY,
            &mask
        );
        XEvent event {};
        event.xclient.type = ClientMessage;
        event.xclient.message_type =
            XInternAtom(display, "_NET_WM_MOVERESIZE", False);
        event.xclient.display = display;
        event.xclient.window = window;
        event.xclient.format = 32;
        event.xclient.data.l[0] = rootX;
        event.xclient.data.l[1] = rootY;
        event.xclient.data.l[2] = direction;
        event.xclient.data.l[3] = Button1;
        event.xclient.data.l[4] = SOURCE_APPLICATION;
        XUngrabPointer(display, CurrentTime);
        XSendEvent(
            display,
            root,
            False,
            SubstructureRedirectMask | SubstructureNotifyMask,
            &event
        );
        XFlush(display);
        return true;
    }
}

#elif defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>

#include <GLFW/glfw3.h>

#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

namespace {
    WNDPROC original_window_proc = nullptr;
    frame::Hit (*hit_test_callback)(void*, int, int) = nullptr;
    void* hit_test_userdata = nullptr;

    LRESULT CALLBACK frame_window_proc(
        HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam
    ) {
        switch (message) {
            case WM_NCCALCSIZE:
                // the caption is gone, the whole window is the client area
                if (wparam == TRUE) {
                    return 0;
                }
                break;
            case WM_NCPAINT:
                // the frame is not drawn by us
                return 0;
            case WM_NCACTIVATE:
                // and it is not repainted when the window is activated
                return TRUE;
            case WM_NCHITTEST: {
                if (hit_test_callback == nullptr) {
                    break;
                }
                POINT point {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
                ScreenToClient(hwnd, &point);
                const frame::Hit hit =
                    hit_test_callback(hit_test_userdata, point.x, point.y);
                if (hit.caption) {
                    return HTCAPTION;
                }
                switch (hit.edges) {
                    case frame::EDGE_TOP | frame::EDGE_LEFT:
                        return HTTOPLEFT;
                    case frame::EDGE_TOP:
                        return HTTOP;
                    case frame::EDGE_TOP | frame::EDGE_RIGHT:
                        return HTTOPRIGHT;
                    case frame::EDGE_RIGHT:
                        return HTRIGHT;
                    case frame::EDGE_BOTTOM | frame::EDGE_RIGHT:
                        return HTBOTTOMRIGHT;
                    case frame::EDGE_BOTTOM:
                        return HTBOTTOM;
                    case frame::EDGE_BOTTOM | frame::EDGE_LEFT:
                        return HTBOTTOMLEFT;
                    case frame::EDGE_LEFT:
                        return HTLEFT;
                    default:
                        return HTCLIENT;
                }
            }
        }
        return CallWindowProc(
            original_window_proc, hwnd, message, wparam, lparam
        );
    }
}
#endif

bool frame_has_native_operations() {
#if defined(VOXEL_X11_FRAME)
    X11Api& api = x11_api();
    return api.available() && x11_supports_moveresize(api.display());
#else
    return false;
#endif
}

bool frame_start_move(GLFWwindow* window) {
#if defined(VOXEL_X11_FRAME)
    return x11_request(window, MOVERESIZE_MOVE);
#else
    (void)window;
    return false;
#endif
}

bool frame_start_resize(GLFWwindow* window, int edges) {
#if defined(VOXEL_X11_FRAME)
    return edges != 0 && x11_request(window, x11_direction(edges));
#else
    (void)window;
    (void)edges;
    return false;
#endif
}

void frame_setup_native(
    GLFWwindow* window,
    frame::Hit (*hitTest)(void*, int, int),
    void* userdata
) {
#if defined(_WIN32)
    HWND hwnd = glfwGetWin32Window(window);
    if (hwnd == nullptr) {
        return;
    }
    hit_test_callback = hitTest;
    hit_test_userdata = userdata;
    // The native frame stays: its thick frame is what gives the window the
    // resize border, the shadow, the animations and the snap layouts. Only
    // the caption is hidden, and the client area covers the whole window.
    LONG_PTR style = GetWindowLongPtr(hwnd, GWL_STYLE);
    style |= WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU;
    style &= ~WS_CAPTION;
    SetWindowLongPtr(hwnd, GWL_STYLE, style);
    original_window_proc = reinterpret_cast<WNDPROC>(SetWindowLongPtr(
        hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(frame_window_proc)
    ));
    SetWindowPos(
        hwnd,
        nullptr,
        0,
        0,
        0,
        0,
        SWP_FRAMECHANGED | SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER
    );
#else
    (void)window;
    (void)hitTest;
    (void)userdata;
#endif
}
