#pragma once

#include <istream>
#include "util/Buffer.hpp"

class memory_streambuf : public std::streambuf {
public:
    explicit memory_streambuf(util::Buffer<char> buffer)
        : buffer(std::move(buffer)) {
        char* base = this->buffer.data();
        char* end = base + this->buffer.size();
        setg(base, base, end);
    }
    
    memory_streambuf(const memory_streambuf&) = delete;
    memory_streambuf& operator=(const memory_streambuf&) = delete;

    pos_type seekoff(off_type off, std::ios_base::seekdir way,
                     std::ios_base::openmode mode = std::ios_base::in) override {
        if (!(mode & std::ios_base::in)) return pos_type(off_type(-1));
        char* base = eback();
        char* end = egptr();
        if (!base || !end) return pos_type(off_type(-1));

        std::ptrdiff_t current = gptr() - base;
        std::ptrdiff_t newpos;
        if (way == std::ios_base::beg) newpos = off;
        else if (way == std::ios_base::cur) newpos = current + off;
        else if (way == std::ios_base::end) newpos = (end - base) + off;
        else return pos_type(off_type(-1));

        if (newpos < 0 || base + newpos > end) return pos_type(off_type(-1));
        setg(base, base + newpos, end);
        return pos_type(off_type(newpos));
    }

    pos_type seekpos(pos_type sp, std::ios_base::openmode mode = std::ios_base::in) override {
        return seekoff(off_type(sp), std::ios_base::beg, mode);
    }
protected:
    int_type underflow() override {
        if (gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        return traits_type::eof();
    }

private:
    util::Buffer<char> buffer;
};

class memory_istream : public std::istream {
public:
    explicit memory_istream(util::Buffer<char> buffer)
        : std::istream(&buf), buf(std::move(buffer)) {}

private:
    memory_streambuf buf;
};

class memory_view_streambuf : public std::streambuf {
public:
    explicit memory_view_streambuf(const util::Buffer<char>& buffer)
        : buffer(buffer) {
        char* base = const_cast<char*>(this->buffer.data());
        char* end = base + this->buffer.size();
        setg(base, base, end);
    }
    
    memory_view_streambuf(const memory_view_streambuf&) = delete;
    memory_view_streambuf& operator=(const memory_view_streambuf&) = delete;

    pos_type seekoff(off_type off, std::ios_base::seekdir way,
                     std::ios_base::openmode mode = std::ios_base::in) override {
        if (!(mode & std::ios_base::in)) return pos_type(off_type(-1));
        char* base = eback();
        char* end = egptr();
        if (!base || !end) return pos_type(off_type(-1));

        std::ptrdiff_t current = gptr() - base;
        std::ptrdiff_t newpos;
        if (way == std::ios_base::beg) newpos = off;
        else if (way == std::ios_base::cur) newpos = current + off;
        else if (way == std::ios_base::end) newpos = (end - base) + off;
        else return pos_type(off_type(-1));

        if (newpos < 0 || base + newpos > end) return pos_type(off_type(-1));
        setg(base, base + newpos, end);
        return pos_type(off_type(newpos));
    }

    pos_type seekpos(pos_type sp, std::ios_base::openmode mode = std::ios_base::in) override {
        return seekoff(off_type(sp), std::ios_base::beg, mode);
    }
protected:
    int_type underflow() override {
        if (gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        return traits_type::eof();
    }

private:
    const util::Buffer<char>& buffer;
};

class memory_view_istream : public std::istream {
public:
    explicit memory_view_istream(const util::Buffer<char>& buffer)
        : std::istream(&buf), buf(buffer) {}

private:
    memory_view_streambuf buf;
};
