#pragma once

#include <memory>

#include "ImageData.hpp"
#include "commons.hpp"
#include "maths/UVRegion.hpp"
#include "typedefs.hpp"

class Texture : public Bindable {
protected:
    uint id;
    uint width;
    uint height;
    uint format;
public:
    Texture(uint id, uint width, uint height, ImageFormat imageFormat);
    Texture(const ubyte* data, uint width, uint height, ImageFormat format);
    virtual ~Texture();

    virtual void bind() const override;
    virtual void unbind() const override;
    virtual void reload(const ubyte* data, uint w, uint h);
    void reloadPartial(const ImageData& image, uint x, uint y, uint w, uint h);

    void setNearestFilter();

    void reload(const ImageData& image);
    virtual void resize(uint w, uint h);

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
