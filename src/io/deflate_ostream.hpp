#pragma once

#include <iostream>
#include <streambuf>
#include <memory>
#include <zlib.h>

class deflate_ostreambuf : public std::streambuf {
public:
    deflate_ostreambuf(std::ostream& dest, int level = Z_DEFAULT_COMPRESSION)
        : dest(dest) {
        zstream.zalloc = Z_NULL;
        zstream.zfree = Z_NULL;
        zstream.opaque = Z_NULL;
        int ret = deflateInit2(
            &zstream, level, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY
        );
        if (ret != Z_OK) {
            throw std::runtime_error("zlib deflate initialization failed");
        }
        inBuffer = std::make_unique<char[]>(bufferSize);
        outBuffer = std::make_unique<char[]>(outBufferSize);

        setp(inBuffer.get(), inBuffer.get() + bufferSize - 1);
    }

    ~deflate_ostreambuf() {
        try {
            close();
        } catch (...) {
            std::cerr << "error in zlib output stream finalization" << std::endl;
        }
    }

    bool close() {
        overflow(EOF);

        // Finalize the deflate stream
        zstream.avail_in = 0;
        zstream.next_in = nullptr;
        int ret;
        do {
            zstream.avail_out = outBufferSize;
            zstream.next_out = reinterpret_cast<Bytef*>(outBuffer.get());
            ret = deflate(&zstream, Z_FINISH);
            if (ret == Z_STREAM_ERROR) {
                break;
            }
            size_t compressed_size = outBufferSize - zstream.avail_out;
            dest.write(outBuffer.get(), compressed_size);
        } while (ret != Z_STREAM_END);

        deflateEnd(&zstream);
        return true;
    }

protected:
    int overflow(int c) override {
        if (c != EOF) {
            *pptr() = static_cast<char>(c);
            pbump(1);
        }

        if (process_input() == EOF) {
            return EOF;
        }

        return c != EOF ? 0 : EOF;
    }

    int sync() override {
        if (process_input(Z_SYNC_FLUSH) == EOF) {
            return -1;
        }
        dest.flush();
        return 0;
    }

private:
    static const size_t bufferSize = 512;
    static const size_t outBufferSize = bufferSize * 2;

    std::ostream& dest;
    z_stream zstream {};
    std::unique_ptr<char[]> inBuffer;
    std::unique_ptr<char[]> outBuffer;

    int process_input(int flush = Z_NO_FLUSH) {
        size_t input_size = pptr() - pbase();
        zstream.avail_in = static_cast<uInt>(input_size);
        zstream.next_in = reinterpret_cast<Bytef*>(pbase());

        int ret;
        do {
            zstream.avail_out = outBufferSize;
            zstream.next_out = reinterpret_cast<Bytef*>(outBuffer.get());
            ret = deflate(&zstream, flush);
            if (ret == Z_STREAM_ERROR) {
                return EOF;
            }
            size_t compressed_size = outBufferSize - zstream.avail_out;
            dest.write(outBuffer.get(), compressed_size);
            if (!dest) {
                return EOF;
            }
        } while (zstream.avail_out == 0);

        setp(inBuffer.get(), inBuffer.get() + bufferSize - 1);
        return 0;
    }
};

class deflate_ostream : public std::ostream {
public:
    explicit deflate_ostream(std::ostream& dest, int level = Z_DEFAULT_COMPRESSION)
        : std::ostream(&buffer), buffer(dest, level) {}

private:
    deflate_ostreambuf buffer;
};
