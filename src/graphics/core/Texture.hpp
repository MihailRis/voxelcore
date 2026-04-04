#pragma once

#include "typedefs.hpp"
#include "maths/UVRegion.hpp"
#include "ImageData.hpp"

#include <memory>

class Texture {
protected:
    uint id;
    uint width;
    uint height;
public:
    Texture(uint id, uint width, uint height);
    Texture(const ubyte* data, uint width, uint height, ImageFormat format);
    virtual ~Texture();

    virtual void bind() const;
    virtual void unbind() const;
    void reload(const ubyte* data, uint w, uint h);
    void reloadPartial(const ImageData& image, uint x, uint y, uint w, uint h);

    void setNearestFilter();

    void reload(const ImageData& image);

    void setMipMapping(bool flag, bool pixelated);

    std::unique_ptr<ImageData> readData();
    uint getId() const;

    UVRegion getUVRegion() const {
        return UVRegion(0.0f, 0.0f, 1.0f, 1.0f);
    }

    uint getWidth() const {
        return width;
    }

    uint getHeight() const {
        return height;
    }

    static std::unique_ptr<Texture> from(const ImageData* image);
    static uint MAX_RESOLUTION;
};
