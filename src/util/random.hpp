#pragma once

#include <random>
#include <string>
#include <algorithm>

namespace util {
    template <
        class T = std::mt19937,
        std::size_t N = T::state_size * sizeof(typename T::result_type)>
    auto seeded_random_engine(std::random_device& source) {
        std::random_device::result_type randomData[(N - 1) / sizeof(source()) + 1];
        std::generate(std::begin(randomData), std::end(randomData), std::ref(source));
        std::seed_seq seeds(std::begin(randomData), std::end(randomData));
        return T(seeds);
    }

    std::string generate_uuid();
}
