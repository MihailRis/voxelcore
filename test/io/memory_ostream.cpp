#include <gtest/gtest.h>

#include "io/memory_ostream.hpp"

TEST(io, memory_ostream) {
    const char data[] = "Hello, world!";
    const int n = std::strlen(data);

    memory_ostream stream;
    ASSERT_TRUE(stream.good());

    stream.write(data, n);
    ASSERT_EQ(std::string(stream.view()), std::string(data));

    stream.seekp(0);
    ASSERT_TRUE(stream.good());
    stream.write("Remove", 6);
    ASSERT_EQ(std::string(stream.view()), "Remove world!");

    auto buffer = stream.release();
    ASSERT_EQ(std::string(buffer.data(), buffer.size()), "Remove world!");
}
