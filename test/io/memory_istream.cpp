#include <gtest/gtest.h>

#include "io/memory_istream.hpp"

TEST(io, memory_istream) {
    const char data[] = "Hello, world!";
    const int n = std::strlen(data);

    util::Buffer<char> buffer(data, n);
    memory_istream stream(std::move(buffer));

    ASSERT_TRUE(stream.good());

    std::string text(n, '\0');
    stream.read(text.data(), n);
    ASSERT_EQ(text, std::string(data));
    stream.read(text.data(), 1);
    ASSERT_TRUE(stream.eof());

    // seek
    stream.clear();
    stream.seekg(0);
    ASSERT_TRUE(stream.good());
    stream.read(text.data(), n);
    ASSERT_EQ(text, std::string(data));
    stream.seekg(-6, std::ios_base::cur);
    ASSERT_TRUE(stream.good());
    stream.read(text.data(), 6);
    ASSERT_EQ(text.substr(0, 6), "world!");
}
