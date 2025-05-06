#pragma once

#include "typedefs.hpp"
#include "commons.hpp"

class GBuffer : public Bindable{
public:
    GBuffer(uint width, uint height);
    ~GBuffer();

    void bind() override;
    void unbind() override;

    void bindBuffers();

    void resize(uint width, uint height);

    uint getWidth() const;
    uint getHeight() const;
private:
    uint width;
    uint height;

    uint fbo;
    uint colorBuffer;
    uint positionsBuffer;
    uint normalsBuffer;
    uint depthBuffer;

    void createColorBuffer();
    void createPositionsBuffer();
    void createNormalsBuffer();
    void createDepthBuffer();
};
