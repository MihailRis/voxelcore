#include "window/wayland/WaylandWindow.hpp"

#include "settings.hpp"

static constexpr int BAR_HEIGHT = 30;
static constexpr int BAR_BUTTON = 30;
static constexpr int BAR_EDGE = 5;
const std::string& WaylandWindow::getTitle() const {
    return title;
}

int WaylandWindow::getDecorationHeight() const {
    return barVisible() ? BAR_HEIGHT : 0;
}

int WaylandWindow::getDecorationHoveredButton() const {
    return hoveredButton;
}

void WaylandWindow::enableOwnDecorations() {
    if (!decorations_enabled()) {
        return;
    }
    barEnabled = true;
    setShouldRefresh();
}

bool WaylandWindow::barVisible() const {
    return barEnabled && !fullscreen && Window::mode != WindowMode::BORDERLESS;
}

int WaylandWindow::barButtonAt(double x, double y) const {
    if (!barVisible() || y < 0.0 || y >= BAR_HEIGHT) {
        return -1;
    }
    for (int i = 0; i < 3; i++) {
        const double bx = size.x - BAR_BUTTON * (i + 1);
        if (x >= bx && x < bx + BAR_BUTTON) {
            return i;
        }
    }
    return -1;
}

uint32_t WaylandWindow::barEdgeAt(double x, double y) const {
    uint32_t edges = 0;
    if (x < BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
    } else if (x >= size.x - BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
    }
    if (barVisible() && y < BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_TOP;
    } else if (y >= size.y - BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
    }
    return edges;
}

CursorShape WaylandWindow::edgeCursor(uint32_t edges) {
    const bool left = edges & XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
    const bool right = edges & XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
    const bool top = edges & XDG_TOPLEVEL_RESIZE_EDGE_TOP;
    const bool bottom = edges & XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
    if ((left && top) || (right && bottom)) {
        return CursorShape::NWSE_RESIZE;
    }
    if ((right && top) || (left && bottom)) {
        return CursorShape::NESW_RESIZE;
    }
    if (left || right) {
        return CursorShape::EW_RESIZE;
    }
    if (top || bottom) {
        return CursorShape::NS_RESIZE;
    }
    return CursorShape::ARROW;
}

bool WaylandWindow::handleBarMotion(double x, double y) {
    pointerX = x;
    pointerY = y;
    const int button = barButtonAt(x, y);
    if (barVisible() && y >= 0.0 && y < BAR_HEIGHT) {
        hoveredButton = button;
        input.setOverrideCursor(
            button >= 0 ? CursorShape::POINTER : CursorShape::ARROW
        );
        return true;
    }
    hoveredButton = -1;
    const uint32_t edges = barEdgeAt(x, y);
    input.setOverrideCursor(
        edges != 0 ? std::optional<CursorShape>(edgeCursor(edges))
                   : std::nullopt
    );
    return false;
}

bool WaylandWindow::handleBarButton(uint32_t serial, bool pressed) {
    const double x = pointerX;
    const double y = pointerY;
    const int button = barButtonAt(x, y);
    if (button >= 0) {
        if (pressed) {
            if (button == 0) {
                setShouldClose(true);
            } else if (button == 1) {
                setMaximized(!maximized);
            } else {
                xdg_toplevel_set_minimized(state.toplevel);
            }
        }
        return true;
    }
    if (barVisible() && y >= 0.0 && y < BAR_HEIGHT) {
        if (pressed) {
            const double current = now();
            if (current - lastBarPress < 0.4) {
                lastBarPress = 0.0;
                setMaximized(!maximized);
            } else {
                lastBarPress = current;
                xdg_toplevel_move(state.toplevel, state.seat, serial);
            }
        }
        return true;
    }
    const uint32_t edges = barEdgeAt(x, y);
    if (pressed && edges != 0) {
        xdg_toplevel_resize(state.toplevel, state.seat, serial, edges);
        return true;
    }
    return false;
}

void WaylandWindow::onPointerLeave() {
    hoveredButton = -1;
    input.setOverrideCursor(std::nullopt);
}
