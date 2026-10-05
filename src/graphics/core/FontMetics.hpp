#pragma once

#include <memory>
#include <string>
#include <optional>

class Font;

struct FontMetrics {
    std::optional<std::weak_ptr<Font>> font;

    int lineHeight = 0;
    int yoffset = 0;
    int _glyphInterval = 8;

    ~FontMetrics();

    int calcWidth(std::wstring_view text, size_t offset=0, size_t length=-1) const;
};
