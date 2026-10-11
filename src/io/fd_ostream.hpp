#pragma once

#include <array>
#include <iostream>
#include <streambuf>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifndef _WIN32
#include <unistd.h>
#endif


class fd_ostreambuf final : public std::streambuf {
    static constexpr size_t BUFFER_SIZE = 4096;
public:
#ifdef _WIN32
    explicit fd_ostreambuf(HANDLE handle) : handle(handle) {
        setp(buffer.data(), buffer.data() + buffer.size());
    }

    ~fd_ostreambuf() override {
        fd_ostreambuf::sync();
        if (handle != nullptr && handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
            handle = INVALID_HANDLE_VALUE;
        }
    }
protected:
    int_type overflow(int_type ch) override {
        if (ch == traits_type::eof()) {
            return traits_type::not_eof(ch);
        }
        if (pptr() == epptr()) {
            if (sync() != 0) {
                return traits_type::eof();
            }
        }
        *pptr() = traits_type::to_char_type(ch);
        pbump(1);
        return ch;
    }

    int sync() override {
        if (handle == nullptr || handle == INVALID_HANDLE_VALUE) {
            return -1;
        }

        const auto size = static_cast<size_t>(pptr() - pbase());
        if (size == 0) {
            return 0;
        }

        DWORD written = 0;
        if (!WriteFile(
                handle, pbase(), static_cast<DWORD>(size), &written, nullptr
            )) {
            return -1;
        }

        setp(buffer.data(), buffer.data() + buffer.size());
        return 0;
    }

    pos_type seekoff(off_type, std::ios_base::seekdir, std::ios_base::openmode)
        override {
        return pos_type(off_type(-1));
    }

    pos_type seekpos(pos_type, std::ios_base::openmode) override {
        return pos_type(off_type(-1));
    }
private:
    HANDLE handle;
    std::array<char, BUFFER_SIZE> buffer {};
#else
    explicit fd_ostreambuf(int fd) : fd(fd) {
        setp(buffer.data(), buffer.data() + buffer.size());
    }

    ~fd_ostreambuf() override {
        fd_ostreambuf::sync();
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
protected:
    int_type overflow(int_type ch) override {
        if (ch == traits_type::eof()) {
            return traits_type::not_eof(ch);
        }
        if (pptr() == epptr()) {
            if (sync() != 0) {
                return traits_type::eof();
            }
        }
        *pptr() = traits_type::to_char_type(ch);
        pbump(1);
        return ch;
    }

    int sync() override {
        if (fd < 0) {
            return -1;
        }

        const auto size = static_cast<size_t>(pptr() - pbase());
        if (size == 0) {
            return 0;
        }

        char* ptr = pbase();
        size_t remaining = size;
        while (remaining > 0) {
            const auto written = ::write(fd, ptr, remaining);
            if (written <= 0) {
                return -1;
            }
            ptr += written;
            remaining -= static_cast<size_t>(written);
        }

        setp(buffer.data(), buffer.data() + buffer.size());
        return 0;
    }

    pos_type seekoff(off_type, std::ios_base::seekdir, std::ios_base::openmode)
        override {
        return pos_type(off_type(-1));
    }

    pos_type seekpos(pos_type, std::ios_base::openmode) override {
        return pos_type(off_type(-1));
    }
private:
    int fd;
    std::array<char, BUFFER_SIZE> buffer {};
#endif
};

class fd_ostream final : public std::ostream {
public:
#ifdef _WIN32
    explicit fd_ostream(HANDLE handle) : std::ostream(&buf), buf(handle) {
    }
#else
    explicit fd_ostream(int fd) : std::ostream(&buf), buf(fd) {
    }
#endif
private:
    fd_ostreambuf buf;
};
