#include <gtest/gtest.h>
#include <string>

#include "util/stack_vector.hpp"

using namespace util;

TEST(util, stack_vector) {
    stack_vector<std::string, 4> vec;
    vec.push_back("hello");
    vec.push_back("world");
    vec.push_back("?");
    ASSERT_EQ(3, vec.size());
    vec.pop_back();
    ASSERT_EQ(2, vec.size());
    vec.clear();
    ASSERT_EQ(0, vec.size());
    ASSERT_TRUE(vec.empty());
    vec.push_back("test");
}
