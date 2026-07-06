#include <gtest/gtest.h>

#include "util/SmallHeap.hpp"

using namespace util;

TEST(SmallHeap, Allocation) {
    auto index = 0;
    auto size = 4;

    std::cout << "step 1" << std::endl;
    SmallHeap<uint16_t, uint8_t> map;
    std::cout << "step 2" << std::endl;
    auto ptr = map.allocate(index, size);
    std::cout << "ptr: " << (void*)ptr << " le2h(1): " << dataio::le2h(1) << " h2le(1): " << dataio::h2le(1) << std::endl;
    EXPECT_EQ(map.sizeOf(ptr), size);
    std::cout << "step 4" << std::endl;
    EXPECT_EQ(ptr, map.find(index));
}

TEST(SmallHeap, MultipleAllocation) {
    SmallHeap<uint16_t, uint8_t> map;
    map.allocate(32, 4);
    map.allocate(1, 5);
    EXPECT_EQ(map.sizeOf(map.find(32)), 4);
    EXPECT_EQ(map.sizeOf(map.find(1)), 5);
}

TEST(SmallHeap, Free) {
    SmallHeap<uint16_t, uint8_t> map;
    map.free(map.allocate(5, 4));
    EXPECT_EQ(map.find(5), nullptr);
}

TEST(SmallHeap, ReAllocationSame) {
    SmallHeap<uint16_t, uint8_t> map;
    auto ptr1 = map.allocate(1, 2);
    auto ptr2 = map.allocate(1, 2);
    EXPECT_EQ(ptr1, ptr2);
}

TEST(SmallHeap, ReAllocationDifferent) {
    SmallHeap<uint16_t, uint8_t> map;
    map.allocate(1, 34);
    map.allocate(1, 2);
    EXPECT_EQ(map.sizeOf(map.find(1)), 2);
}

TEST(SmallHeap, RandomFill) {
    SmallHeap<uint16_t, uint8_t> map;
    int n = 3'000;
    map.allocate(n, 123);
    for (int i = 0; i < n; i++) {
        int index = rand() % n;
        int size = rand() % 254 + 1;
        map.allocate(index, size);
    }
    EXPECT_EQ(map.sizeOf(map.find(n)), 123);
}

TEST(SmallHeap, EncodeDecode) {
    SmallHeap<uint16_t, uint8_t> map;
    int n = 3'000;
    map.allocate(n, 123);
    for (int i = 0; i < n; i++) {
        int index = rand() % n;
        int size = rand() % 254 + 1;
        map.allocate(index, size);
    }
    auto bytes = map.serialize();

    SmallHeap<uint16_t, uint8_t> out;
    out.deserialize(bytes.data(), bytes.size());
    EXPECT_EQ(map, out);
}

TEST(SmallHeap, Iterator) {
    SmallHeap<uint16_t, uint8_t> map;
    map.allocate(1, 10);
    map.allocate(2, 20);
    map.allocate(4, 14);
    
    int sum = 0;
    for (const auto& it : map) {
        sum += it.size();
    }
    EXPECT_EQ(sum, 44);
}
