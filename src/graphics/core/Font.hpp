#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <glm/glm.hpp>

#include "typedefs.hpp"
#include "FontMetics.hpp"
#include "data/dv_fwd.hpp"
#include "../commons/FontStyle.hpp"

class Texture;
class Batch2D;
class Batch3D;
class Camera;
class ImageData;

class Font;

namespace vector_fonts {
    class FontFile;
}

struct Glyph {
    int yOffset;
    int xAdvance;
};

class Font {
public:
    Font(
        std::vector<std::unique_ptr<Texture>> pages,
        std::vector<Glyph> glyphs,
        int lineHeight,
        int yoffset,
        std::optional<std::weak_ptr<vector_fonts::FontFile>> fontFile = std::nullopt
    );
    ~Font();

    int getLineHeight() const;
    int getYOffset() const;
    
    /// @brief Calculate text width in pixels
    /// @param text selected text
    /// @param length max substring length (default: no limit)
    /// @return pixel width of the substring 
    int calcWidth(std::wstring_view text, size_t length=-1) const;

    /// @brief Calculate text width in pixels
    /// @param text selected text
    /// @param offset start of the substring
    /// @param length max substring length
    /// @return pixel width of the substring
    int calcWidth(std::wstring_view text, size_t offset, size_t length) const;

    /// @brief Check if character is visible (non-whitespace)
    /// @param codepoint character unicode codepoint
    bool isPrintableChar(uint codepoint) const;

    void draw(
        Batch2D& batch,
        std::wstring_view text,
        int x,
        int y,
        const FontStylesScheme* styles,
        size_t styleMapOffset,
        float scale = 1
    );

    void draw(
        Batch3D& batch,
        std::wstring_view text,
        const FontStylesScheme* styles,
        size_t styleMapOffset,
        const glm::vec3& pos,
        const glm::vec3& right={1, 0, 0},
        const glm::vec3& up={0, 1, 0}
    );

    const Texture* getPage(int page) const;

    FontMetrics getMetrics() const {
        return {std::nullopt, lineHeight, yoffset, glyphInterval};
    }
 
    const Glyph* getGlyph(int codepoint);

    static std::unique_ptr<Font> createBitmapFont(
        std::vector<std::unique_ptr<ImageData>> pages
    );
private:
    int lineHeight;
    int yoffset;
    int glyphInterval;
    std::vector<std::unique_ptr<Texture>> pages;
    std::vector<Glyph> glyphs;
    std::optional<std::weak_ptr<vector_fonts::FontFile>> fontFile;
};
