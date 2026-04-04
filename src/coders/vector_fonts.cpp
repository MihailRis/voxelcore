#include "vector_fonts.hpp"

#include "io/io.hpp"
#include "debug/Logger.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/Font.hpp"
#include "constants.hpp"

#include <stdexcept>
#include <ft2build.h>
#include FT_FREETYPE_H

static debug::Logger logger("vector_fonts");

static FT_Library library;

static const char* get_ft_error_message(FT_Error error) {
    #undef FTERRORS_H_
    #define FT_ERRORDEF( e, v, s )  case v: return s;
    #define FT_ERROR_START_LIST     switch (error) {
    #define FT_ERROR_END_LIST       }
    #include FT_ERRORS_H
    return "unknown error";
}

namespace {
    class FTFontFile : public vector_fonts::FontFile,
                       public std::enable_shared_from_this<FTFontFile> {
    public:
        FTFontFile(FT_Face face, util::Buffer<ubyte> buffer)
            : face(std::move(face)), buffer(std::move(buffer)) {
        }

        ~FTFontFile() {
            if (FT_Error error = FT_Done_Face(face)) {
                logger.error()
                    << "error on FT_Done_Face: " << get_ft_error_message(error);
            }
        }

        std::unique_ptr<Font> createInstance(int size) override {
            if (FT_Error error = FT_Set_Pixel_Sizes(face, 0, size)) {
                throw std::runtime_error(
                    "FT_Set_Pixel_Sizes error: " +
                    std::string(get_ft_error_message(error))
                );
            }

            std::vector<std::unique_ptr<Texture>> pages;
            std::vector<Glyph> glyphs;

            int dstSize = size * 16;
            ImageData canvas(ImageFormat::RGBA8888, dstSize, dstSize);
            ImageData bitmapDst(ImageFormat::RGBA8888, size, size);

            for (int pageid = 0; pageid < DEFAULT_PRE_RENDER_FONT_PAGES; pageid++) {
                renderPage(pageid, canvas, bitmapDst, glyphs);
                pages.push_back(Texture::from(&canvas));
            }
            return std::make_unique<Font>(
                std::move(pages),
                std::move(glyphs),
                size,
                size / 2,
                shared_from_this()
            );
        }

        std::unique_ptr<Texture> renderPage(
            int pageid, std::vector<Glyph>& glyphs, int size
        ) override {
            if (FT_Error error = FT_Set_Pixel_Sizes(face, 0, size)) {
                throw std::runtime_error(
                    "FT_Set_Pixel_Sizes error: " +
                    std::string(get_ft_error_message(error))
                );
            }

            int dstSize = size * 16;
            ImageData canvas(ImageFormat::RGBA8888, dstSize, dstSize);
            ImageData bitmapDst(ImageFormat::RGBA8888, size, size);

            renderPage(pageid, canvas, bitmapDst, glyphs);
            return Texture::from(&canvas);
        }
    private:
        FT_Face face;
        util::Buffer<ubyte> buffer;

        bool renderGlyph(int codepoint, ImageData& bitmapDst) {
            int width = bitmapDst.getWidth();
            int height = bitmapDst.getHeight();

            if (FT_Error error = FT_Load_Char(face, codepoint, FT_LOAD_RENDER)) {
                logger.warning() << get_ft_error_message(error);
                return false;
            }
            auto dstData = bitmapDst.getData();
            std::memset(dstData, 0, bitmapDst.getDataSize());
            if (face->glyph->bitmap.width == 0) {
                return false;
            }
            const FT_Bitmap& bitmap = face->glyph->bitmap;
            for (int row = 0; row < std::min<int>(bitmap.rows, height); row++) {
                for (int col = 0; col < std::min<int>(bitmap.width, width); col++) {
                    uint8_t value = bitmap.buffer[row * bitmap.pitch + col];
                    dstData[(row * width + col) * 4 + 0] = 255;
                    dstData[(row * width + col) * 4 + 1] = 255;
                    dstData[(row * width + col) * 4 + 2] = 255;
                    dstData[(row * width + col) * 4 + 3] = value;
                }
            }
            return true;
        }

        void renderPage(
            int pageid,
            ImageData& canvas,
            ImageData& bitmapDst,
            std::vector<Glyph>& glyphs
        ) {
            int size = canvas.getWidth() / 16;
            for (int c = 0; c < 256; c++) {
                int codepoint = pageid << 8 | c;
                if (!renderGlyph(codepoint, bitmapDst)) {
                    glyphs.push_back(Glyph {0, size / 2});
                    continue;
                }

                canvas.blit(bitmapDst, (c % 16) * size, (c / 16) * size);
                
                Glyph glyph {
                    face->glyph->bitmap_top - size,
                    static_cast<int>(face->glyph->advance.x >> 6)
                };
                if (codepoint < glyphs.size()) {
                    glyphs[codepoint] = std::move(glyph);
                } else {
                    glyphs.push_back(std::move(glyph));
                }
            }
            canvas.flipY();
        }
    };
}

void vector_fonts::initialize() {
    if (FT_Error error = FT_Init_FreeType(&library)) {
        throw std::runtime_error(
            "could not initialize freetype: " +
            std::string(get_ft_error_message(error))
        );
    }
}

void vector_fonts::finalize() {
    if (FT_Error error = FT_Done_FreeType(library)) {
        logger.error() << "error on FT_Done_FreeType: "
                       << get_ft_error_message(error);
    }
}

std::shared_ptr<vector_fonts::FontFile> vector_fonts::load_font(
    const std::string& filename
) {
    auto bytes = io::read_bytes_buffer(io::path(filename));

    FT_Face face;

    if (FT_Error error =
            FT_New_Memory_Face(library, bytes.data(), bytes.size(), 0, &face)) {
        throw std::runtime_error(
            "could not load font '" + filename +
            "': " + std::string(get_ft_error_message(error))
        );
    }

    return std::make_shared<FTFontFile>(std::move(face), std::move(bytes));
}
