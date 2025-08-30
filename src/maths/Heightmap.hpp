#pragma once

#include <vector>
#include <string>
#include <optional>

#include "typedefs.hpp"
#include "maths/Heightmap.hpp"
#include "util/EnumMetadata.hpp"

enum class InterpolationType {
    NEAREST,
    LINEAR,
    CUBIC,
};

VC_ENUM_METADATA(InterpolationType)
    {"nearest", InterpolationType::NEAREST},
    {"linear", InterpolationType::LINEAR},
    {"cubic", InterpolationType::CUBIC},
VC_ENUM_END

class Heightmap {
    uint width, height;
    std::vector<float> buffer;
public:
    Heightmap(uint width, uint height)
        : width(width), height(height) {
        buffer.resize(width*height);
    }

    Heightmap(uint width, uint height, std::vector<float> buffer) 
    : width(width), height(height), buffer(std::move(buffer)) {}

    ~Heightmap() = default;

    void resize(uint width, uint height, InterpolationType interpolation);

    void crop(uint srcX, uint srcY, uint dstWidth, uint dstHeight);

    void clamp();

    uint getWidth() const {
        return width;
    }

    uint getHeight() const {
        return height;
    }

    float get(uint x, uint y) {
        return buffer.at(y * width + x);
    }

    float getUnchecked(uint x, uint y) {
        return buffer[y * width + x];
    }

    float* getValues() {
        return buffer.data();
    }

    const float* getValues() const {
        return buffer.data();
    }
};
