#include "Heightmap.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <glm/glm.hpp>

static inline float sample_at(
    const float* buffer,
    uint width,
    uint x, uint y
) {
    return buffer[y*width+x];
}

static inline float sample_at(
    const float* buffer,
    uint width, uint height,
    uint x, uint y
) {
    return buffer[(y >= height ? height-1 : y)*width+(x >= width ? width-1 : x)];
}

static inline float interpolate_cubic(float p[4], float x) {
    return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - 
           p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
}

static inline float interpolate_bicubic(float p[4][4], float x, float y) {
    float q[4];
    for (int i = 0; i < 4; i++) {
        q[i] = interpolate_cubic(p[i], y);
    }
    return interpolate_cubic(q, x);
}

static inline float sample_at(
    const float* buffer,
    uint width, uint height,
    float x, float y,
    InterpolationType interp
) {
    // std::floor is redundant here because x and y are positive values
    uint ix = static_cast<uint>(x);
    uint iy = static_cast<uint>(y);
    float val = buffer[iy*width+ix];
    if (interp == InterpolationType::NEAREST) {
        return val;
    }
    float tx = x - ix;
    float ty = y - iy;

    switch (interp) {
        case InterpolationType::LINEAR: {
            float s00 = val;
            float s10 = sample_at(buffer, width, 
                ix + 1 < width ? ix + 1 : ix, iy);
            float s01 = sample_at(buffer, width, ix, 
                iy + 1 < height ? iy + 1 : iy);
            float s11 = sample_at(buffer, width, 
                ix + 1 < width ? ix + 1 : ix, iy + 1 < height ? iy + 1 : iy);

            float a00 = s00;
            float a10 = s10 - s00;
            float a01 = s01 - s00;
            float a11 = s11 - s10 - s01 + s00;

            return a00 + a10*tx + a01*ty + a11*tx*ty;
        }
        case InterpolationType::CUBIC: {
            float p[4][4];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    p[i][j] = sample_at(
                        buffer, width, height, ix + j - 1, iy + i - 1
                    );
                }
            }
            return interpolate_bicubic(p, ty, tx);
        }
        default:
            throw std::runtime_error("interpolation type is not implemented");
    }
    return val;
}

void Heightmap::resize(
    uint dstwidth, uint dstheight, InterpolationType interp
) {
    if (width == dstwidth && height == dstheight) {
        return;
    }
    std::vector<float> dst;
    dst.resize(dstwidth*dstheight);

    uint index = 0;
    for (uint y = 0; y < dstheight; y++) {
        for (uint x = 0; x < dstwidth; x++, index++) {
            float sx = static_cast<float>(x) / dstwidth * width;
            float sy = static_cast<float>(y) / dstheight * height;
            dst[index] = sample_at(buffer.data(), width, height, sx, sy, interp);
        }
    }

    width = dstwidth;
    height = dstheight;
    buffer = std::move(dst);
}

void Heightmap::crop(
    uint srcx, uint srcy, uint dstwidth, uint dstheight
) {
    if (srcx + dstwidth > width || srcy + dstheight > height) {
        throw std::runtime_error(
            "crop zone is not fully inside of the source image");
    }
    if (dstwidth == width && dstheight == height) {
        return;
    }

    std::vector<float> dst;
    dst.resize(dstwidth*dstheight);

    for (uint y = 0; y < dstheight; y++) {
        std::memcpy(
            dst.data()+y*dstwidth, 
            buffer.data()+(y+srcy)*width+srcx, 
            dstwidth*sizeof(float));
    }

    width = dstwidth;
    height = dstheight;
    buffer = std::move(dst);
}

void Heightmap::clamp() {
    for (uint i = 0; i < width * height; i++) {
        buffer[i] = std::min(1.0f, std::max(0.0f, buffer[i]));
    }
}
