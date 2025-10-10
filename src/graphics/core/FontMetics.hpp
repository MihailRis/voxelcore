#pragma once

#include <string>

struct FontMetrics {
    int lineHeight;
    int yoffset;
    int glyphInterval = 8;

    int calcWidth(std::wstring_view text, size_t offset=0, size_t length=-1) const;
};
