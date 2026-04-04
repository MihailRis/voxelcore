#include "rle.hpp"

#include <stdexcept>

#include "util/data_io.hpp"

size_t rle::decode(const ubyte* src, size_t srclen, ubyte* dst, size_t dstLength) {
    size_t offset = 0;
    for (size_t i = 0; i < srclen;) {
        ubyte len = src[i++];
        ubyte c = src[i++];
        if (offset + len >= dstLength) {
            throw std::runtime_error("buffer overflow");
        }
        for (size_t j = 0; j <= len; j++) {
            dst[offset++] = c;
        }
    }
    return offset;
}

size_t rle::encode(const ubyte* src, size_t srclen, ubyte* dst) {
    if (srclen == 0) {
        return 0;
    }
    size_t offset = 0;
    ubyte counter = 0;
    ubyte c = src[0];
    for (size_t i = 1; i < srclen; i++) {
        ubyte cnext = src[i];
        if (cnext != c || counter == 255) {
            dst[offset++] = counter;
            dst[offset++] = c;
            c = cnext;
            counter = 0;
        } else {
            counter++;
        }
    }
    dst[offset++] = counter;
    dst[offset++] = c;
    return offset;
}

size_t rle::decode16(const ubyte* src, size_t srclen, ubyte* dst, size_t dstLength) {
    auto src16 = reinterpret_cast<const uint16_t*>(src);
    auto dst16 = reinterpret_cast<uint16_t*>(dst);
    size_t offset = 0;
    for (size_t i = 0; i < srclen / 2;) {
        uint16_t len = dataio::le2h(src16[i++]);
        uint16_t c = dataio::le2h(src16[i++]);
        if (offset + len >= dstLength) {
            throw std::runtime_error("buffer overflow");
        }
        for (size_t j = 0; j <= len; j++) {
            dst16[offset++] = c;
        }
    }
    return offset * 2;
}

size_t rle::encode16(const ubyte* src, size_t srclen, ubyte* dst) {
    if (srclen == 0) {
        return 0;
    }
    auto src16 = reinterpret_cast<const uint16_t*>(src);
    auto dst16 = reinterpret_cast<uint16_t*>(dst);
    size_t offset = 0;
    uint16_t counter = 0;
    uint16_t c = src16[0];
    for (size_t i = 1; i < srclen / 2; i++) {
        uint16_t cnext = src16[i];
        if (cnext != c || counter == 0xFFFF) {
            dst16[offset++] = dataio::h2le(counter);
            dst16[offset++] = dataio::h2le(c);
            c = cnext;
            counter = 0;
        } else {
            counter++;
        }
    }
    dst16[offset++] = dataio::h2le(counter);
    dst16[offset++] = dataio::h2le(c);
    return offset * 2;
}

size_t extrle::decode(const ubyte* src, size_t srclen, ubyte* dst, size_t dstLength) {
    size_t offset = 0;
    for (size_t i = 0; i < srclen;) {
        uint len = src[i++];
        if (len & 0x80) {
            len &= 0x7F;
            len |= (static_cast<uint>(src[i++])) << 7;
        }
        ubyte c = src[i++];
        if (offset + len >= dstLength) {
            throw std::runtime_error("buffer overflow");
        }
        for (size_t j = 0; j <= len; j++) {
            dst[offset++] = c;
        }
    }
    return offset;
}

size_t extrle::encode(const ubyte* src, size_t srclen, ubyte* dst) {
    if (srclen == 0) {
        return 0;
    }
    size_t offset = 0;
    uint counter = 0;
    ubyte c = src[0];
    for (size_t i = 1; i < srclen; i++) {
        ubyte cnext = src[i];
        if (cnext != c || counter == max_sequence) {
            if (counter >= 0x80) {
                dst[offset++] = 0x80 | (counter & 0x7F);
                dst[offset++] = counter >> 7;
            } else {
                dst[offset++] = counter;
            }
            dst[offset++] = c;
            c = cnext;
            counter = 0;
        } else {
            counter++;
        }
    }
    if (counter >= 0x80) {
        dst[offset++] = 0x80 | (counter & 0x7F);
        dst[offset++] = counter >> 7;
    } else {
        dst[offset++] = counter;
    }
    dst[offset++] = c;
    return offset;
}

size_t extrle::decode16(const ubyte* src, size_t srclen, ubyte* dst8, size_t dstLength) {
    auto dst = reinterpret_cast<uint16_t*>(dst8);
    size_t offset = 0;
    for (size_t i = 0; i < srclen;) {
        uint len = src[i++];
        bool widechar = len & 0x40;
        if (len & 0x80) {
            len &= 0x3F;
            len |= (static_cast<uint>(src[i++])) << 6;
        } else {
            len &= 0x3F;
        }
        uint16_t c = src[i++];
        if (widechar) {
            c |= ((static_cast<uint>(src[i++])) << 8);
        }
        if (offset + len >= dstLength) {
            throw std::runtime_error("buffer overflow");
        }
        for (size_t j = 0; j <= len; j++) {
            dst[offset++] = c;
        }
    }
    return offset * 2;
}

size_t extrle::encode16(const ubyte* src8, size_t srclen, ubyte* dst) {
    if (srclen == 0) {
        return 0;
    }
    auto src = reinterpret_cast<const uint16_t*>(src8);
    size_t offset = 0;
    uint counter = 0;
    uint16_t c = src[0];
    for (size_t i = 1; i < srclen/2; i++) {
        uint16_t cnext = src[i];
        if (cnext != c || counter == max_sequence16) {
            if (counter >= 0x40) {
                dst[offset++] = 0x80 | ((c > 255) << 6) | (counter & 0x3F);
                dst[offset++] = counter >> 6;
            } else {
                dst[offset++] = counter | ((c > 255) << 6);
            }
            if (c > 255) {
                dst[offset++] = c & 0xFF;
                dst[offset++] = c >> 8;
            } else {
                dst[offset++] = c;
            }
            c = cnext;
            counter = 0;
        } else {
            counter++;
        }
    }
    if (counter >= 0x40) {
        dst[offset++] = 0x80 | ((c > 255) << 6) | (counter & 0x3F);
        dst[offset++] = counter >> 6;
    } else {
        dst[offset++] = counter | ((c > 255) << 6);
    }
    if (c > 255) {
        dst[offset++] = c & 0xFF;
        dst[offset++] = c >> 8;
    } else {
        dst[offset++] = c;
    }
    return offset;
}
