#pragma once

#include <array>
#include <iostream>
#include <streambuf>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#endif

#ifndef _WIN32
#include <unistd.h>
#endif

class fd_istreambuf final : public std::streambuf {
    static constexpr size_t BUFFER_SIZE = 4096;
public:
#ifdef _WIN32
    explicit fd_istreambuf(HANDLE handle)
        : handle(handle) {
        setg(buffer.data(), buffer.data(), buffer.data());
    }

    ~fd_istreambuf() override {
        if (handle != nullptr && handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
            handle = INVALID_HANDLE_VALUE;
        }
    }

protected:
    int_type underflow() override {
        if (handle == nullptr || handle == INVALID_HANDLE_VALUE) {
            return traits_type::eof();
        }
        if (gptr() != egptr()) {
            return traits_type::to_int_type(*gptr());
        }

        std::array<char, BUFFER_SIZE> chunk {};
        DWORD readCount = 0;
        if (!ReadFile(
                handle,
                chunk.data(),
                static_cast<DWORD>(chunk.size()),
                &readCount,
                nullptr
            ) ||
            readCount == 0) {
            return traits_type::eof();
        }

        buffer.assign(chunk.begin(), chunk.begin() + readCount);
        setg(buffer.data(), buffer.data(), buffer.data() + buffer.size());
        return traits_type::to_int_type(*gptr());
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
    std::vector<char> buffer;
#else
    explicit fd_istreambuf(int fd)
        : fd(fd) {
        setg(buffer.data(), buffer.data(), buffer.data());
    }

    ~fd_istreambuf() override {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

protected:
    int_type underflow() override {
        if (fd < 0) {
            return traits_type::eof();
        }
        if (gptr() != egptr()) {
            return traits_type::to_int_type(*gptr());
        }

        buffer.resize(BUFFER_SIZE);
        const auto count = ::read(fd, buffer.data(), buffer.size());
        if (count <= 0) {
            return traits_type::eof();
        }
        buffer.resize(static_cast<size_t>(count));
        setg(buffer.data(), buffer.data(), buffer.data() + buffer.size());
        return traits_type::to_int_type(*gptr());
    }

    pos_type seekoff(
        off_type,
        std::ios_base::seekdir,
        std::ios_base::openmode
    ) override {
        return pos_type(off_type(-1));
    }

    pos_type seekpos(pos_type, std::ios_base::openmode) override {
        return pos_type(off_type(-1));
    }

private:
    int fd;
    std::vector<char> buffer;
#endif
};

class fd_istream final : public std::istream {
public:
#ifdef _WIN32
    explicit fd_istream(HANDLE handle)
        : std::istream(&buf), buf(handle) {}
#else
    explicit fd_istream(int fd)
        : std::istream(&buf), buf(fd) {}
#endif

private:
    fd_istreambuf buf;
};
