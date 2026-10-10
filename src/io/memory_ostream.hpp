#pragma once

#include <iostream>
#include <streambuf>
#include <cstring>
#include <climits>
#include <memory>
#include <algorithm>
#include <string_view>

#include "util/Buffer.hpp"

class memory_buffer : public std::streambuf {
public:
    explicit memory_buffer(size_t initial_size = 64)
        : buffer(std::make_unique<char[]>(initial_size)),
          capacity(initial_size) {
        setp(buffer.get(), buffer.get() + capacity);
    }

    std::string_view view() const {
        return std::string_view(pbase(), size());
    }

    util::Buffer<char> release() {
        size_t n = size();
        return {std::move(buffer), n};
    }

    size_t size() const {
        return std::max(length, position());
    }

    void set_sync_callback(std::function<void(std::string_view)> callback) {
        on_sync = std::move(callback);
    }
protected:
    int_type overflow(int_type c) override {
        if (c == traits_type::eof())
            return traits_type::eof();

        reserve(position() + 1);
        *pptr() = traits_type::to_char_type(c);
        pbump(1);
        return c;
    }

    std::streamsize xsputn(const char* s, std::streamsize count) override {
        reserve(position() + count);
        std::memcpy(pptr(), s, count);
        advance(count);
        return count;
    }

    pos_type seekoff(
        off_type off,
        std::ios_base::seekdir dir,
        std::ios_base::openmode which
    ) override {
        if (!(which & std::ios_base::out)) {
            return pos_type(off_type(-1));
        }
        length = size();

        off_type base;
        switch (dir) {
            case std::ios_base::beg: base = 0; break;
            case std::ios_base::cur: base = position(); break;
            case std::ios_base::end: base = length; break;
            default: return pos_type(off_type(-1));
        }

        off_type target = base + off;
        if (target < 0 || target > static_cast<off_type>(length)) {
            return pos_type(off_type(-1));
        }
        set_position(target);
        return pos_type(target);
    }

    pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
        return seekoff(off_type(pos), std::ios_base::beg, which);
    }
private:
    std::unique_ptr<char[]> buffer;
    size_t capacity;
    size_t length = 0;

    size_t position() const {
        return pptr() - pbase();
    }

    void advance(size_t n) {
        while (n > 0) {
            int step = static_cast<int>(std::min<size_t>(n, INT_MAX));
            pbump(step);
            n -= step;
        }
    }

    void set_position(size_t pos) {
        setp(buffer.get(), buffer.get() + capacity);
        advance(pos);
    }

    void reserve(size_t required) {
        if (required <= capacity) {
            return;
        }
        size_t pos = position();
        length = size();

        size_t new_capacity = std::max(capacity * 2, required);
        auto new_buffer = std::make_unique<char[]>(new_capacity);
        std::memcpy(new_buffer.get(), buffer.get(), length);

        buffer = std::move(new_buffer);
        capacity = new_capacity;
        set_position(pos);
    }

    int sync() override {
        if (on_sync) {
            on_sync(view());
        }
        return 0;
    }

    std::function<void(std::string_view)> on_sync;
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

    void set_sync_callback(std::function<void(std::string_view)> callback) {
        buffer.set_sync_callback(std::move(callback));
    }
private:
    memory_buffer buffer;
};
