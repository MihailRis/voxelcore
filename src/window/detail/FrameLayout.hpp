#pragma once

/// @brief Geometry of the window frame the engine draws itself.
///
/// Wayland has its own implementation because it also talks to the compositor,
/// this one is used by the backends that decorate their window by themselves.
/// It deliberately depends on nothing but the standard library, so the layout
/// and the hit testing can be checked without a window.
namespace frame {
    constexpr int BAR_HEIGHT = 48;
    constexpr int BAR_HEIGHT_COMPACT = 24;
    constexpr int BUTTON = 24;
    constexpr int PADDING = 12;
    constexpr int GAP = 12;
    constexpr int EDGE = 8;
    constexpr int MAX_BUTTONS = 4;

    constexpr int EDGE_TOP = 1;
    constexpr int EDGE_BOTTOM = 2;
    constexpr int EDGE_LEFT = 4;
    constexpr int EDGE_RIGHT = 8;

    struct Rect {
        float x = 0.0f;
        float width = 0.0f;
    };

    /// @brief Places the controls of the bar flush against the right edge.
    /// @param count number of controls, ordered from left to right
    /// @return number of placed controls, which is 0 for a window too narrow
    inline int layoutButtons(
        float windowWidth, int count, bool compact, Rect* out
    ) {
        if (count > MAX_BUTTONS) {
            count = MAX_BUTTONS;
        }
        const float padding = compact ? 0.0f : PADDING;
        const float gap = compact ? 0.0f : GAP;
        const float pitch = BUTTON + gap;
        int placed = 0;
        for (int i = 0; i < count; i++) {
            const float x = windowWidth - padding - (count - i) * pitch + gap;
            if (x < 0.0f) {
                break;
            }
            out[placed++] = {x, static_cast<float>(BUTTON)};
        }
        return placed;
    }

    /// @brief Control under the pointer, or -1
    inline int buttonAt(
        const Rect* buttons, int count, int barHeight, float x, float y
    ) {
        const int top = (barHeight - BUTTON) / 2;
        if (y < top || y >= top + BUTTON) {
            return -1;
        }
        for (int i = 0; i < count; i++) {
            if (x >= buttons[i].x && x < buttons[i].x + buttons[i].width) {
                return i;
            }
        }
        return -1;
    }

    /// @brief What the pointer is over, in client coordinates
    struct Hit {
        /// the bar drags the window, which on win32 is the OS caption
        bool caption = false;
        /// resize edges, 0 when the pointer is not near any of them
        int edges = 0;
    };

    /// @brief Resize edges grab-able at the given position, 0 when none
    inline int edgesAt(
        int width, int height, int barHeight, float x, float y
    ) {
        int edges = 0;
        if (y < EDGE) {
            edges |= EDGE_TOP;
        } else if (y >= height - EDGE) {
            edges |= EDGE_BOTTOM;
        }
        // Inside the bar only its top strip resizes the window, the rest
        // belongs to the controls and to dragging.
        if (y < barHeight && y >= EDGE) {
            return edges;
        }
        if (x < EDGE) {
            edges |= EDGE_LEFT;
        } else if (x >= width - EDGE) {
            edges |= EDGE_RIGHT;
        }
        return edges;
    }
}
