#include "window/wayland/WaylandWindow.hpp"

#include "settings.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
    constexpr const char* LAYOUT_SCHEMA = "org.gnome.desktop.wm.preferences";
    constexpr const char* LAYOUT_KEY = "button-layout";

    std::vector<DecorationButton> parse_buttons(const std::string& text) {
        std::vector<DecorationButton> buttons;
        size_t start = 0;
        while (start <= text.size()) {
            const size_t end = text.find(',', start);
            const std::string name = text.substr(
                start,
                end == std::string::npos ? std::string::npos : end - start
            );
            if (name == "close") {
                buttons.push_back(DecorationButton::CLOSE);
            } else if (name == "minimize" || name == "min") {
                buttons.push_back(DecorationButton::MINIMIZE);
            } else if (name == "maximize" || name == "max") {
                buttons.push_back(DecorationButton::MAXIMIZE);
            }
            if (end == std::string::npos) {
                break;
            }
            start = end + 1;
        }
        return buttons;
    }

    void split_layout(
        const std::string& value, std::vector<DecorationButton>& left,
        std::vector<DecorationButton>& right
    ) {
        std::string text = value;
        text.erase(
            std::remove_if(text.begin(), text.end(), [] (char c) {
                return c == '\'' || c == '"' || c == '\n' || c == '\r';
            }),
            text.end()
        );
        const size_t separator = text.find(':');
        if (separator == std::string::npos) {
            left.clear();
            right = parse_buttons(text);
        } else {
            left = parse_buttons(text.substr(0, separator));
            right = parse_buttons(text.substr(separator + 1));
        }
    }

    bool read_button_layout(
        std::vector<DecorationButton>& left,
        std::vector<DecorationButton>& right
    ) {
        FILE* pipe = popen(
            "gsettings get org.gnome.desktop.wm.preferences button-layout"
            " 2>/dev/null",
            "r"
        );
        if (pipe == nullptr) {
            return false;
        }
        char buffer[256] {};
        const char* line = fgets(buffer, sizeof(buffer), pipe);
        const int status = pclose(pipe);
        if (line == nullptr || status != 0) {
            return false;
        }
        split_layout(std::string(buffer), left, right);
        return true;
    }

    constexpr double DRAG_THRESHOLD = 5.0;
    constexpr double LAYOUT_CHECK_INTERVAL = 2.0;
    constexpr double DOUBLE_CLICK_TIME = 0.4;
    constexpr double DOUBLE_CLICK_DISTANCE = 8.0;
}

void WaylandWindow::applyLayoutValue(const std::string& rawValue) {
    std::vector<DecorationButton> left;
    std::vector<DecorationButton> right;
    split_layout(rawValue, left, right);
    if (left == leftButtons && right == rightButtons) {
        return;
    }
    leftButtons = std::move(left);
    rightButtons = std::move(right);
    updateBarGeometry();
    setShouldRefresh();
}

void WaylandWindow::updateButtonLayout() {
    if (!layoutRead) {
        layoutRead = true;
        std::string initial;
        if (settingsWatcher.read(LAYOUT_SCHEMA, LAYOUT_KEY, initial)) {
            applyLayoutValue(initial);
        } else {
            std::vector<DecorationButton> left;
            std::vector<DecorationButton> right;
            if (read_button_layout(left, right)) {
                leftButtons = std::move(left);
                rightButtons = std::move(right);
                updateBarGeometry();
            }
        }
    }
    std::string value;
    if (settingsWatcher.poll(LAYOUT_SCHEMA, LAYOUT_KEY, value)) {
        applyLayoutValue(value);
        return;
    }
    if (settingsWatcher.isAvailable()) {
        return;
    }
    const double current = now();
    if (current - lastLayoutCheck < LAYOUT_CHECK_INTERVAL) {
        return;
    }
    lastLayoutCheck = current;
    std::vector<DecorationButton> left;
    std::vector<DecorationButton> right;
    if (!read_button_layout(left, right)) {
        return;
    }
    if (left == leftButtons && right == rightButtons) {
        return;
    }
    leftButtons = std::move(left);
    rightButtons = std::move(right);
    updateBarGeometry();
}

void WaylandWindow::updateBarGeometry() {
    if (leftButtons.empty() && rightButtons.empty()) {
        // desktop layout is unavailable, fall back to the usual three controls
        rightButtons = {
            DecorationButton::MINIMIZE,
            DecorationButton::MAXIMIZE,
            DecorationButton::CLOSE,
        };
    }
    barButtons.clear();
    const float width = static_cast<float>(size.x);
    // The narrow bar has no room for the insets of the wide one, so its
    // controls sit flush against the window edge and each other.
    const float padding = compactBar() ? 0.0f : BAR_PADDING;
    const float gap = compactBar() ? 0.0f : BAR_GAP;
    const float pitch = BAR_BUTTON + gap;
    for (size_t i = 0; i < leftButtons.size(); i++) {
        const float x = padding + static_cast<float>(i) * pitch;
        if (x + BAR_BUTTON > width) {
            break;
        }
        barButtons.push_back({leftButtons[i], x, BAR_BUTTON});
    }
    for (size_t i = 0; i < rightButtons.size(); i++) {
        const float x = width - padding -
                        static_cast<float>(rightButtons.size() - i) * pitch +
                        gap;
        if (x < 0.0f) {
            break;
        }
        barButtons.push_back({rightButtons[i], x, BAR_BUTTON});
    }
}

const std::vector<DecorationButtonLayout>&
WaylandWindow::getDecorationButtons() const {
    return barButtons;
}

const std::string& WaylandWindow::getTitle() const {
    return title;
}

int WaylandWindow::getDecorationHeight() const {
    return barVisible() ? barHeight() : 0;
}

int WaylandWindow::getDecorationHoveredButton() const {
    return hoveredButton;
}

void WaylandWindow::enableOwnDecorations() {
    barEnabled = true;
    setShouldRefresh();
}

void WaylandWindow::disableOwnDecorations() {
    if (!barEnabled) {
        return;
    }
    barEnabled = false;
    resize(size.x, size.y);
    setShouldRefresh();
}

bool WaylandWindow::barVisible() const {
    return barEnabled && !fullscreen && Window::mode != WindowMode::BORDERLESS;
}

int WaylandWindow::barButtonAt(double x, double y) const {
    const int top = (barHeight() - BAR_BUTTON) / 2;
    if (!barVisible() || y < top || y >= top + BAR_BUTTON) {
        return -1;
    }
    for (size_t i = 0; i < barButtons.size(); i++) {
        const auto& item = barButtons[i];
        if (x >= item.x && x < item.x + item.width) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

uint32_t WaylandWindow::barEdgeAt(double x, double y) const {
    if (barButtonAt(x, y) >= 0) {
        return 0;
    }
    const bool inBar = barVisible() && y >= 0.0 && y < barHeight();
    uint32_t edges = 0;
    if (y < BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_TOP;
    } else if (y >= size.y - BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
    }
    if (inBar && y >= BAR_EDGE) {
        return edges;
    }
    if (x < BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
    } else if (x >= size.x - BAR_EDGE) {
        edges |= XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
    }
    return edges;
}

CursorShape WaylandWindow::edgeCursor(uint32_t edges) {
    const bool left = edges & XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
    const bool right = edges & XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
    const bool top = edges & XDG_TOPLEVEL_RESIZE_EDGE_TOP;
    const bool bottom = edges & XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
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

uint32_t WaylandWindow::pointerEdges() const {
    const int margin = marginSize();
    uint32_t edges = 0;
    if (margin > 0) {
        const auto surface = surfaceSize();
        const int edge = margin + RESIZE_GRAB_EXTRA;
        if (pointerX < edge) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_LEFT;
        } else if (pointerX >= surface.x - edge) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_RIGHT;
        }
        if (pointerY < edge) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_TOP;
        } else if (pointerY >= surface.y - edge) {
            edges |= XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM;
        }
    }
    if (edges == 0 && margin == 0) {
        edges = barEdgeAt(contentX(pointerX), contentY(pointerY));
    }
    return edges;
}

bool WaylandWindow::handleBarMotion(double x, double y) {
    pointerX = x;
    pointerY = y;
    if (barPressed && !barDragging &&
        (std::abs(x - barPressX) > DRAG_THRESHOLD ||
         std::abs(y - barPressY) > DRAG_THRESHOLD)) {
        barDragging = true;
        hoveredButton = -1;
        xdg_toplevel_move(state.toplevel, state.seat, barPressSerial);
    }
    const uint32_t edges = pointerEdges();
    if (edges != 0) {
        hoveredButton = -1;
        pointerInBar = false;
        input.setOverrideCursor(edgeCursor(edges));
        return true;
    }
    const double contentY = this->contentY(y);
    const bool inBar =
        barVisible() && contentY >= 0.0 && contentY < barHeight();
    if (inBar && !pointerInBar) {
        lastLayoutCheck = 0.0;
    }
    pointerInBar = inBar;
    if (inBar) {
        hoveredButton =
            barDragging ? -1 : barButtonAt(contentX(x), contentY);
        input.setOverrideCursor(
            hoveredButton >= 0 ? CursorShape::POINTER : CursorShape::ARROW
        );
        return true;
    }
    hoveredButton = -1;
    input.setOverrideCursor(std::nullopt);
    return false;
}

bool WaylandWindow::handleBarButton(uint32_t serial, int button, bool pressed) {
    const double x = contentX(pointerX);
    const double y = contentY(pointerY);
    const uint32_t edges = pointerEdges();
    const bool inBar = barVisible() && y >= 0.0 && y < barHeight();
    if (button != 0) {
        return inBar || edges != 0;
    }
    if (edges != 0) {
        if (pressed) {
            xdg_toplevel_resize(state.toplevel, state.seat, serial, edges);
        }
        return true;
    }
    if (!inBar) {
        return false;
    }
    if (pressed) {
        const double current = now();
        const bool doubleClick =
            current - lastBarPress < DOUBLE_CLICK_TIME &&
            std::abs(x - lastBarPressX) < DOUBLE_CLICK_DISTANCE &&
            std::abs(y - lastBarPressY) < DOUBLE_CLICK_DISTANCE;
        lastBarPress = current;
        lastBarPressX = x;
        lastBarPressY = y;
        if (doubleClick) {
            lastBarPress = 0.0;
            setMaximized(!maximized);
            return true;
        }
        barPressed = true;
        barDragging = false;
        barPressedButton = barButtonAt(x, y);
        barPressX = x;
        barPressY = y;
        barPressSerial = serial;
        hoveredButton = barPressedButton;
        return true;
    }
    const int releasedButton = barPressedButton;
    const bool dragged = barDragging;
    barPressed = false;
    barDragging = false;
    barPressedButton = -1;
    if (!dragged && releasedButton >= 0 &&
        releasedButton < static_cast<int>(barButtons.size()) &&
        barButtonAt(x, y) == releasedButton) {
        switch (barButtons[releasedButton].button) {
            case DecorationButton::CLOSE:
                setShouldClose(true);
                break;
            case DecorationButton::MAXIMIZE:
                setMaximized(!maximized);
                break;
            case DecorationButton::MINIMIZE:
                xdg_toplevel_set_minimized(state.toplevel);
                break;
        }
    }
    return true;
}

bool WaylandWindow::handleBarScroll() const {
    const double y = contentY(pointerY);
    return (barVisible() && y >= 0.0 && y < barHeight()) ||
           pointerEdges() != 0;
}

void WaylandWindow::onPointerLeave() {
    hoveredButton = -1;
    pointerInBar = false;
    barPressed = false;
    barDragging = false;
    barPressedButton = -1;
    input.setOverrideCursor(std::nullopt);
}
