#pragma once

#include "typedefs.hpp"

#include <glm/vec4.hpp>
#include <memory>

enum class ImageFormat {
    rgb888,
    rgba8888
};

class ImageData {
    ImageFormat format;
    uint width;
    uint height;
    std::unique_ptr<ubyte[]> data;

    void blitRGB_on_RGBA(const ImageData& image, int x, int y);
    void blitMatchingFormat(const ImageData& image, int x, int y);
public:
    ImageData(ImageFormat format, uint width, uint height);
    ImageData(ImageFormat format, uint width, uint height, std::unique_ptr<ubyte[]> data);
    ImageData(ImageFormat format, uint width, uint height, const ubyte* data);
    ~ImageData();

    void flipX();
    void flipY();

    void drawLine(int x1, int y1, int x2, int y2, const glm::ivec4& color);
    void drawRect(int x, int y, int width, int height, const glm::ivec4& color);
    void blit(const ImageData& image, int x, int y);
    void extrude(int x, int y, int w, int h);
    void fixAlphaColor();

    ubyte* getData() const {
        return data.get();
    }

    ImageFormat getFormat() const {
        return format;
    }

    uint getWidth() const {
        return width;
    }

    uint getHeight() const {
        return height;
    }

    size_t getDataSize() const {
        size_t channels = 3 + (format == ImageFormat::rgba8888);
        return width * height * channels;
    }
};

std::unique_ptr<ImageData> add_atlas_margins(ImageData* image, int grid_size);
