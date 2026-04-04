#pragma once

#include "data/dv_fwd.hpp"

#include <vector>
#include <glm/glm.hpp>

struct FontStyle {
    bool bold = false;
    bool italic = false;
    bool strikethrough = false;
    bool underline = false;
    glm::vec4 color {1, 1, 1, 1};

    FontStyle() = default;

    FontStyle(
        bool bold,
        bool italic,
        bool strikethrough,
        bool underline,
        glm::vec4 color
    )
        : bold(bold),
          italic(italic),
          strikethrough(strikethrough),
          underline(underline),
          color(std::move(color)) {
    }

    static FontStyle parse(const dv::value& src);
};

struct FontStylesScheme {
    std::vector<FontStyle> palette;
    std::vector<unsigned char> map;

    static FontStylesScheme parse(const dv::value& src);
};
