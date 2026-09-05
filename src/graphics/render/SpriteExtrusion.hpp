#pragma once

#include <vector>

/// @brief One straight run of pixels along the edge of a sprite's solid part.
///
/// A run travels along one axis and sits at one line of the other, so four
/// numbers say the whole of it. `outwardX`/`outwardY` is which way the
/// outside is, and exactly one of the two is ever non-zero.
struct SpriteEdge {
    int outwardX = 0;
    int outwardY = 0;
    /// The row a horizontal run is on, or the column a vertical one is in.
    int line = 0;
    /// The pixels the run covers along its own axis, both ends included.
    int from = 0;
    int to = 0;

    bool operator==(const SpriteEdge& o) const {
        return outwardX == o.outwardX && outwardY == o.outwardY &&
               line == o.line && from == o.from && to == o.to;
    }
};

/// @brief Where a sprite needs a side if it is given a thickness.
///
/// A sprite drawn as a front and a back has nothing to show edge-on but
/// the seam between them, which is the artefact: two faces on one plane,
/// fighting over the same depth. Giving it a thickness only helps if the
/// rim is closed, and the rim follows the shape the sprite actually is -
/// which is where a pixel that is drawn touches one that is not, the edge
/// of the image counting as not drawn.
///
/// Consecutive pixels wanting the same edge come back as one run rather
/// than one each, so a plain square sprite is four quads and not sixty.
/// Holes inside the sprite get their rim from the same rule, facing in.
///
/// @param opaque one entry per pixel, row by row from the bottom
/// @param width pixels across, must divide `opaque` exactly
std::vector<SpriteEdge> find_sprite_edges(
    const std::vector<bool>& opaque, int width
);
