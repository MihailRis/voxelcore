#include <gtest/gtest.h>

#include "typedefs.hpp"
#include "coders/rle.hpp"

static void test_encode_decode(
    size_t(*encodefunc)(const ubyte*, size_t, ubyte*),
    size_t(*decodefunc)(const ubyte*, size_t, ubyte*, size_t),
    int dencity
) {
    const size_t initial_size = 50'000;
    uint8_t initial[initial_size];
    uint8_t next = rand();
    for (size_t i = 0; i < initial_size; i++) {
        initial[i] = next;
        if (rand() % dencity == 0) {
            next = rand();
        }
    }
    uint8_t encoded[initial_size * 2];
    size_t encoded_size = encodefunc(initial, initial_size, encoded);
    uint8_t decoded[initial_size * 2];
    size_t decoded_size = decodefunc(encoded, encoded_size, decoded, initial_size);
    
    EXPECT_EQ(decoded_size, initial_size);

    for (size_t i = 0; i < decoded_size; i++) {
        EXPECT_EQ(decoded[i], initial[i]);
    }
}

TEST(RLE, EncodeDecode) {
    test_encode_decode(rle::encode, rle::decode, 13);
    test_encode_decode(rle::encode, rle::decode, 90123);
}

TEST(RLE16, EncodeDecode) {
    test_encode_decode(rle::encode16, rle::decode16, 13);
    test_encode_decode(rle::encode16, rle::decode16, 90123);
}

TEST(ExtRLE, EncodeDecode) {
    test_encode_decode(extrle::encode, extrle::decode, 13);
    test_encode_decode(extrle::encode, extrle::decode, 90123);
}

TEST(ExtRLE16, EncodeDecode) {
    test_encode_decode(extrle::encode16, extrle::decode16, 13);
    test_encode_decode(extrle::encode16, extrle::decode16, 90123);
}
