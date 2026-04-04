#pragma once

#include "typedefs.hpp"

namespace rle {
    size_t encode(const ubyte* src, size_t length, ubyte* dst);
    size_t decode(const ubyte* src, size_t length, ubyte* dst, size_t dstLength);

    size_t encode16(const ubyte* src, size_t length, ubyte* dst);
    size_t decode16(const ubyte* src, size_t length, ubyte* dst, size_t dstLength);
}

namespace extrle {
    constexpr uint max_sequence = 0x7FFF;
    size_t encode(const ubyte* src, size_t length, ubyte* dst);
    size_t decode(const ubyte* src, size_t length, ubyte* dst, size_t dstLength);

    constexpr uint max_sequence16 = 0x3FFF;
    size_t encode16(const ubyte* src, size_t length, ubyte* dst);
    size_t decode16(const ubyte* src, size_t length, ubyte* dst, size_t dstLength);
}
