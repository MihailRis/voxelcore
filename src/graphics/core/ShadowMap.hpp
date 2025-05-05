#pragma once

#include "typedefs.hpp"

class ShadowMap {
public:
    ShadowMap(int resolution);
    ~ShadowMap();

    void bind();
    void unbind();
    uint getDepthMap();
private:
    uint fbo;
    uint depthMap; 
};
