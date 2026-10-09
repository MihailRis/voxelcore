#include "window/wayland/WaylandWindow.hpp"

#include "settings.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <sys/wait.h>
#include <unistd.h>

namespace {
    constexpr const char* LAYOUT_SCHEMA = "org.gnome.desktop.wm.preferences";
    constexpr const char* LAYOUT_KEY = "button-layout";

    std::vector<DecorationButton> parse_buttons(const std::string& text) {
        std::vector<DecorationButton> buttons;
        size_t start = 0;
        while (start <= text.size()) {
            const size_t end = text.find(',', start);
            const std::string name = text.substr(
                start, end == std::string::npos ? std::string::npos : end - start
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

    bool read_button_layout(
        std::vector<DecorationButton>& left, std::vector<DecorationButton>& right
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
        std::string value(buffer);
        value.erase(
            std::remove_if(value.begin(), value.end(), [] (char c) {
                return c == '\'' || c == '"' || c == '\n' || c == '\r';
            }),
            value.end()
        );
        const size_t separator = value.find(':');
        if (separator == std::string::npos) {
            left.clear();
            right = parse_buttons(value);
        } else {
            left = parse_buttons(value.substr(0, separator));
            right = parse_buttons(value.substr(separator + 1));
        }
        return true;
    }

    constexpr double DRAG_THRESHOLD = 5.0;
    constexpr double DOUBLE_CLICK_TIME = 0.4;
    constexpr double DOUBLE_CLICK_DISTANCE = 8.0;
}

void WaylandWindow::applyLayoutValue(const std::string& rawValue) {
    std::string value = rawValue;
    value.erase(
        std::remove_if(value.begin(), value.end(), [] (char c) {
            return c == '\'' || c == '"' || c == '\n' || c == '\r';
        }),
        value.end()
    );
    const size_t separator = value.find(':');
    std::vector<DecorationButton> left;
    std::vector<DecorationButton> right;
    if (separator == std::string::npos) {
        right = parse_buttons(value);
    } else {
        left = parse_buttons(value.substr(0, separator));
        right = parse_buttons(value.substr(separator + 1));
    }
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
    if (current - lastLayoutCheck < 2.0) {
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
    barButtons.clear();
    const float width = static_cast<float>(size.x);
    for (size_t i = 0; i < leftButtons.size(); i++) {
        const float x = static_cast<float>(i) * BAR_BUTTON;
        if (x + BAR_BUTTON > width) {
            break;
        }
        barButtons.push_back({leftButtons[i], x, BAR_BUTTON});
    }
    for (size_t i = 0; i < rightButtons.size(); i++) {
        const float x = width -
                        static_cast<float>(rightButtons.size() - i) * BAR_BUTTON;
        if (x < 0.0f) {
            break;
        }
        barButtons.push_back({rightButtons[i], x, BAR_BUTTON});
    }
}

const std::vector<DecorationButtonLayout>& WaylandWindow::getDecorationButtons() const {
    return barButtons;
}

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
    const bool inBar = barVisible() && y >= 0.0 && y < BAR_HEIGHT;
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
    if (barPressed && !barDragging &&
        (std::abs(x - barPressX) > DRAG_THRESHOLD ||
         std::abs(y - barPressY) > DRAG_THRESHOLD)) {
        barDragging = true;
        hoveredButton = -1;
        xdg_toplevel_move(state.toplevel, state.seat, barPressSerial);
    }
    const uint32_t edges = barEdgeAt(x, y);
    if (edges != 0) {
        hoveredButton = -1;
        pointerInBar = false;
        input.setOverrideCursor(edgeCursor(edges));
        return true;
    }
    const bool inBar = barVisible() && y >= 0.0 && y < BAR_HEIGHT;
    if (inBar && !pointerInBar) {
        lastLayoutCheck = 0.0;
    }
    pointerInBar = inBar;
    if (inBar) {
        hoveredButton = barDragging ? -1 : barButtonAt(x, y);
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
    const double x = pointerX;
    const double y = pointerY;
    const uint32_t edges = barEdgeAt(x, y);
    const bool inBar = barVisible() && y >= 0.0 && y < BAR_HEIGHT;
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
    return (barVisible() && pointerY >= 0.0 && pointerY < BAR_HEIGHT) ||
           barEdgeAt(pointerX, pointerY) != 0;
}

void WaylandWindow::onPointerLeave() {
    hoveredButton = -1;
    pointerInBar = false;
    barPressed = false;
    barDragging = false;
    barPressedButton = -1;
    input.setOverrideCursor(std::nullopt);
}
