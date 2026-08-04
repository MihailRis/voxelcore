#pragma once

#include "typedefs.hpp"
#include "commons.hpp"
#include "ImageData.hpp"

class GBuffer : public Bindable {
public:
    GBuffer(uint width, uint height);
    ~GBuffer() override;

    void bind() const override;
    void bindSSAO() const;
    void unbind() const override;

    void bindBuffers() const;
    void bindSSAOBuffer() const;

    void bindDepthBuffer(int drawFbo);

    void resize(uint width, uint height);

    uint getWidth() const;
    uint getHeight() const;

    std::unique_ptr<ImageData> toImage() const;
private:
    struct Buffer {
        uint buffer;
        const int internalformat;
        const uint format;
        const uint type;
        const uint attachment;
    };

    uint width;
    uint height;

    uint fbo;
    Buffer colorBuffer;
    Buffer positionsBuffer;
    Buffer normalsBuffer;
    Buffer emissionBuffer;
    Buffer depthBuffer;
    uint ssaoFbo;
    Buffer ssaoBuffer;

    void createBuffer(Buffer& buffer);
};
