#pragma once

#include <memory>
#include <string>
#include <vector>

class Font;
struct Glyph;

class Texture;

namespace vector_fonts {
    class FontFile {
    public:
        virtual ~FontFile() {}
    
        virtual std::unique_ptr<Font> createInstance(int size) = 0;

        virtual std::unique_ptr<Texture> renderPage(
            int pageid, std::vector<Glyph>& glyphs, int size
        ) = 0;
    };

    void initialize();
    void finalize();

    std::shared_ptr<vector_fonts::FontFile> load_font(const std::string& filename);
}
