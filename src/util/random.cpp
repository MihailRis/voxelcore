#include "random.hpp"

#include <random>

static std::random_device random_device;

static const char* uuid_hex_chars = "0123456789abcdef";
static const char* uuid_hex_variant_chars = "89ab";

std::string util::generate_uuid() {
    auto randomEngine = seeded_random_engine(random_device);
    static std::uniform_int_distribution<> dist(0, 15);
    static std::uniform_int_distribution<> dist2(0, 3);

    std::string uuid;
    uuid.resize(36);

    for (int i = 0; i < 8; i++) {
        uuid[i] = uuid_hex_chars[dist(randomEngine)];
    }
    uuid[8] = '-';
    for (int i = 9; i < 13; i++) {
        uuid[i] = uuid_hex_chars[dist(randomEngine)];
    }
    uuid[13] = '-';
    uuid[14] = '4';

    for (int i = 15; i < 18; i++) {
        uuid[i] = uuid_hex_chars[dist(randomEngine)];
    }

    uuid[18] = '-';
    uuid[19] = uuid_hex_variant_chars[dist2(randomEngine)];

    for (int i = 20; i < 23; i++) {
        uuid[i] = uuid_hex_chars[dist(randomEngine)];
    }

    uuid[23] = '-';
        for (int i = 24; i < 36; i++) {
        uuid[i] = uuid_hex_chars[dist(randomEngine)];
    }
    return uuid;
}
