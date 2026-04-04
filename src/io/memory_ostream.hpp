#pragma once

#include <iostream>
#include <streambuf>
#include <cstring>
#include <algorithm>
#include <string_view>

#include "util/Buffer.hpp"

class memory_buffer : public std::streambuf {
public:
    explicit memory_buffer(size_t initial_size = 64)
        : buffer(std::make_unique<char[]>(initial_size)),
          capacity(initial_size) {
        setp(buffer.get(), buffer.get() + initial_size);
    }

    std::string_view view() const {
        return std::string_view(pbase(), pptr() - pbase());
    }

    util::Buffer<char> release() {
        return {std::move(buffer), size()};
    }

    size_t size() const {
        return pptr() - pbase();
    }
protected:
    int_type overflow(int_type c) override {
        if (c == traits_type::eof())
            return traits_type::eof();

        const size_t data_size = pptr() - pbase();
        const size_t new_capacity = std::max(capacity * 2, data_size + 1);
        auto new_buffer = std::make_unique<char[]>(new_capacity);

        std::memcpy(new_buffer.get(), pbase(), data_size);

        buffer = std::move(new_buffer);
        capacity = new_capacity;

        setp(buffer.get(), buffer.get() + new_capacity);
        pbump(data_size);

        *pptr() = traits_type::to_char_type(c);
        pbump(1);

        return c;
    }

    std::streamsize xsputn(const char* s, std::streamsize count) override {
        const std::streamsize avail = epptr() - pptr();

        if (avail >= count) {
            std::memcpy(pptr(), s, count);
            pbump(count);
            return count;
        }

        std::streamsize written = 0;
        if (avail > 0) {
            std::memcpy(pptr(), s, avail);
            written += avail;
            s += avail;
            count -= avail;
            pbump(avail);
        }

        const size_t data_size = pptr() - pbase();
        const size_t required_capacity = data_size + count;
        const size_t new_capacity = std::max(capacity * 2, required_capacity);
        auto new_buffer = std::make_unique<char[]>(new_capacity);

        std::memcpy(new_buffer.get(), pbase(), data_size);
        std::memcpy(new_buffer.get() + data_size, s, count);

        buffer = std::move(new_buffer);
        capacity = new_capacity;

        setp(buffer.get(), buffer.get() + new_capacity);
        pbump(data_size + count);
        written += count;

        return written;
    }
private:
    std::unique_ptr<char[]> buffer;
    size_t capacity;
};

class memory_ostream : public std::ostream {
public:
    explicit memory_ostream(size_t initialCapacity = 64)
        : std::ostream(&buffer), buffer(initialCapacity) {}

    std::string_view view() const {
        return buffer.view();
    }

    util::Buffer<char> release() {
        return buffer.release();
    }
private:
    memory_buffer buffer;
};
