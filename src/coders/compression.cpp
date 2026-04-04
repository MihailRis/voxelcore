#include "compression.hpp"

#include <string>
#include <cstring>
#include <stdexcept>

#include "rle.hpp"
#include "gzip.hpp"
#include "util/BufferPool.hpp"

using namespace compression;

inline constexpr float BUFFER_NOCROP_THRESOLD = 0.9;

static util::BufferPool<ubyte> buffer_pools[] {
    {255},
    {UINT16_MAX},
    {UINT16_MAX * 8},
};

static std::shared_ptr<ubyte[]> get_buffer(size_t minSize) {
    for (auto& pool : buffer_pools) {
        if (minSize <= pool.getBufferSize()) {
            return pool.get();
        }
    }
    return nullptr;
}

static auto compress_rle(
    const ubyte* src,
    size_t srclen,
    size_t& len,
    size_t(*encodefunc)(const ubyte*, size_t, ubyte*)
) {
    size_t bufferSize = srclen * 2;
    auto buffer = get_buffer(bufferSize);
    auto bytes = buffer.get();
    std::unique_ptr<ubyte[]> uptr;
    if (bytes == nullptr) {
        uptr = std::make_unique<ubyte[]>(bufferSize);
        bytes = uptr.get();
    }
    len = encodefunc(src, srclen, bytes);
    if (uptr) {
        if (len < bufferSize * BUFFER_NOCROP_THRESOLD) {
            auto cropped = std::make_unique<ubyte[]>(len);
            std::memcpy(cropped.get(), uptr.get(), len);
            return cropped;
        }
        return uptr;
    }
    auto data = std::make_unique<ubyte[]>(len);
    std::memcpy(data.get(), bytes, len);
    return data;
}

std::unique_ptr<ubyte[]> compression::compress(
    const ubyte* src, size_t srclen, size_t& len, Method method
) {
    switch (method) {
        case Method::NONE:
            throw std::invalid_argument("compression method is NONE");
        case Method::EXTRLE8:
            return compress_rle(src, srclen, len, extrle::encode);
        case Method::EXTRLE16:
            return compress_rle(src, srclen, len, extrle::encode16);
        case Method::GZIP: {
            auto buffer = gzip::compress(src, srclen);
            auto data = std::make_unique<ubyte[]>(buffer.size());
            std::memcpy(data.get(), buffer.data(), buffer.size());
            len = buffer.size();
            return data;
        }
        default:
            throw std::runtime_error("not implemented");
    }
}

std::unique_ptr<ubyte[]> compression::decompress(
    const ubyte* src, size_t srclen, size_t dstlen, Method method
) {
    switch (method) {
        case Method::NONE:
            throw std::invalid_argument("compression method is NONE");
        case Method::EXTRLE8: {
            auto decompressed = std::make_unique<ubyte[]>(dstlen);
            extrle::decode(src, srclen, decompressed.get(), dstlen);
            return decompressed;
        }
        case Method::EXTRLE16: {
            auto decompressed = std::make_unique<ubyte[]>(dstlen);
            size_t decoded = extrle::decode16(src, srclen, decompressed.get(), dstlen);
            if (decoded != dstlen) {
                throw std::runtime_error(
                    "expected decompressed size " + std::to_string(dstlen) +
                    " got " + std::to_string(decoded));
            }
            return decompressed;
        }
        case Method::GZIP: {
            auto buffer = gzip::decompress(src, srclen);
            if (buffer.size() != dstlen) {
                throw std::runtime_error(
                    "expected decompressed size " + std::to_string(dstlen) +
                    " got " + std::to_string(buffer.size()));
            }
            auto decompressed = std::make_unique<ubyte[]>(buffer.size());
            std::memcpy(decompressed.get(), buffer.data(), buffer.size());
            return decompressed;
        }
        default:
            throw std::runtime_error("method not implemented");
    }
}

void compression::decompress(const util::span<ubyte> src, ubyte* dst, size_t dstlen, Method method) {
    switch (method) {
        case Method::NONE:
            throw std::invalid_argument("compression method is NONE");
        case Method::EXTRLE8:
            extrle::decode(src.data(), src.size(), dst, dstlen);
            break;
        case Method::EXTRLE16: {
            size_t decoded =
                extrle::decode16(src.data(), src.size(), dst, dstlen);
            if (decoded != dstlen) {
                throw std::runtime_error(
                    "expected decompressed size " + std::to_string(dstlen) +
                    " got " + std::to_string(decoded)
                );
            }
            break;
        }
        case Method::GZIP: {
            auto buffer = gzip::decompress(src.data(), src.size());
            if (buffer.size() != dstlen) {
                throw std::runtime_error(
                    "expected decompressed size " + std::to_string(dstlen) +
                    " got " + std::to_string(buffer.size())
                );
            }
            std::memcpy(dst, buffer.data(), buffer.size());
            break;
        }
        default:
            throw std::runtime_error("method not implemented");
    }
}
