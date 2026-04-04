#pragma once

#include <memory>

#include "typedefs.hpp"
#include "util/span.hpp"

namespace compression {
    enum class Method {
        NONE, EXTRLE8, EXTRLE16, GZIP
    };

    /// @brief Compress buffer
    /// @param src source buffer
    /// @param srclen length of the source buffer
    /// @param len (out argument) length of result buffer
    /// @param method compression method
    /// @return compressed bytes array
    /// @throws std::invalid_argument if compression method is NONE
    std::unique_ptr<ubyte[]> compress(
        const ubyte* src, size_t srclen, size_t& len, Method method
    );

    /// @brief Decompress buffer
    /// @param src compressed buffer
    /// @param srclen length of compressed buffer
    /// @param dstlen max expected length of source buffer
    /// @return decompressed bytes array
    std::unique_ptr<ubyte[]> decompress(
        const ubyte* src, size_t srclen, size_t dstlen, Method method);

    void decompress(
        const util::span<ubyte> src, ubyte* dst, size_t dstlen, Method method
    );
}
