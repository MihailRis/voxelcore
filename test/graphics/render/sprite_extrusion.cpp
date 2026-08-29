#include <gtest/gtest.h>

#include <algorithm>

#include "graphics/render/SpriteExtrusion.hpp"

/// Reads a picture the way it looks: the first string is the top row, so
/// the test can be looked at rather than decoded.
static std::vector<bool> picture(std::initializer_list<const char*> rows) {
    std::vector<bool> opaque;
    for (auto it = std::rbegin(rows); it != std::rend(rows); ++it) {
        for (const char* p = *it; *p; p++) {
            opaque.push_back(*p != '.');
        }
    }
    return opaque;
}

static bool has(
    const std::vector<SpriteEdge>& edges,
    int outwardX, int outwardY, int line, int from, int to
) {
    SpriteEdge wanted {outwardX, outwardY, line, from, to};
    return std::find(edges.begin(), edges.end(), wanted) != edges.end();
}

TEST(SpriteExtrusion, OnePixelIsRimmedOnEverySide) {
    auto edges = find_sprite_edges(picture({"#"}), 1);
    ASSERT_EQ(edges.size(), 4);
    EXPECT_TRUE(has(edges, 0, 1, 0, 0, 0));
    EXPECT_TRUE(has(edges, 0, -1, 0, 0, 0));
    EXPECT_TRUE(has(edges, 1, 0, 0, 0, 0));
    EXPECT_TRUE(has(edges, -1, 0, 0, 0, 0));
}

TEST(SpriteExtrusion, ASolidSquareIsFourRunsAndNotSixteen) {
    auto edges = find_sprite_edges(
        picture({"####", "####", "####", "####"}), 4
    );
    EXPECT_EQ(edges.size(), 4);
    EXPECT_TRUE(has(edges, 0, 1, 3, 0, 3));
    EXPECT_TRUE(has(edges, 0, -1, 0, 0, 3));
    EXPECT_TRUE(has(edges, 1, 0, 3, 0, 3));
    EXPECT_TRUE(has(edges, -1, 0, 0, 0, 3));
}

TEST(SpriteExtrusion, NothingDrawnNeedsNoRim) {
    EXPECT_TRUE(find_sprite_edges(picture({"..", ".."}), 2).empty());
}

TEST(SpriteExtrusion, AGapBreaksARunInTwo) {
    auto edges = find_sprite_edges(picture({"##.##"}), 5);
    EXPECT_TRUE(has(edges, 0, 1, 0, 0, 1));
    EXPECT_TRUE(has(edges, 0, 1, 0, 3, 4));
    EXPECT_FALSE(has(edges, 0, 1, 0, 0, 4));
}

TEST(SpriteExtrusion, AHoleGetsARimFacingIntoIt) {
    auto edges = find_sprite_edges(
        picture({"###", "#.#", "###"}), 3
    );
    // The pixel under the hole wants a top edge, the one over it a bottom
    // one, and the two beside it want the sides - all pointing inwards.
    EXPECT_TRUE(has(edges, 0, 1, 0, 1, 1));
    EXPECT_TRUE(has(edges, 0, -1, 2, 1, 1));
    EXPECT_TRUE(has(edges, 1, 0, 0, 1, 1));
    EXPECT_TRUE(has(edges, -1, 0, 2, 1, 1));
}

TEST(SpriteExtrusion, AnLIsRimmedAlongEveryStep) {
    auto edges = find_sprite_edges(picture({"#.", "#.", "##"}), 2);
    EXPECT_TRUE(has(edges, 0, 1, 2, 0, 0));
    EXPECT_TRUE(has(edges, 0, 1, 0, 1, 1));
    EXPECT_TRUE(has(edges, 1, 0, 0, 1, 2));
    EXPECT_TRUE(has(edges, 1, 0, 1, 0, 0));
    EXPECT_TRUE(has(edges, -1, 0, 0, 0, 2));
    EXPECT_TRUE(has(edges, 0, -1, 0, 0, 1));
}

TEST(SpriteExtrusion, ARaggedSizeIsRefusedRatherThanRead) {
    std::vector<bool> five(5, true);
    EXPECT_TRUE(find_sprite_edges(five, 2).empty());
    EXPECT_TRUE(find_sprite_edges(five, 0).empty());
    EXPECT_TRUE(find_sprite_edges({}, 4).empty());
}
