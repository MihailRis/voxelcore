#include "SpriteExtrusion.hpp"

std::vector<SpriteEdge> find_sprite_edges(
    const std::vector<bool>& opaque, int width
) {
    std::vector<SpriteEdge> edges;
    if (width <= 0 || opaque.empty() || opaque.size() % width != 0) {
        return edges;
    }
    const int height = static_cast<int>(opaque.size()) / width;

    // Off the image is off the sprite: an item drawn to the very border of
    // its own texture is still an item with a rim, not one welded to the
    // next square of the atlas.
    //
    // Spelled `-> bool` because indexing a vector<bool> gives a proxy and
    // not a bool, and a lambda whose two returns disagree about their type
    // is not a lambda every compiler will take.
    auto solid = [&](int x, int y) -> bool {
        if (x < 0 || y < 0 || x >= width || y >= height) {
            return false;
        }
        return opaque[static_cast<size_t>(y) * width + x];
    };
    // A run is open while the pixels keep asking for the same edge, and is
    // written down the moment they stop.
    auto sweep = [&](int outwardX, int outwardY, bool horizontal) {
        const int lines = horizontal ? height : width;
        const int along = horizontal ? width : height;
        for (int line = 0; line < lines; line++) {
            int from = -1;
            for (int i = 0; i <= along; i++) {
                const int x = horizontal ? i : line;
                const int y = horizontal ? line : i;
                const bool wants = i < along && solid(x, y) &&
                                   !solid(x + outwardX, y + outwardY);
                if (wants && from < 0) {
                    from = i;
                } else if (!wants && from >= 0) {
                    edges.push_back(
                        SpriteEdge {outwardX, outwardY, line, from, i - 1}
                    );
                    from = -1;
                }
            }
        }
    };
    sweep(0, 1, true);
    sweep(0, -1, true);
    sweep(1, 0, false);
    sweep(-1, 0, false);
    return edges;
}
