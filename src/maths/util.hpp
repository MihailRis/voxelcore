#pragma once

#include <stdint.h>

#include <ctime>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

namespace util {
    inline uint64_t shuffle_bits_step(uint64_t x, uint64_t m, unsigned shift) {
        uint64_t t = ((x >> shift) ^ x) & m;
        x = (x ^ t) ^ (t << shift);
        return x;
    }

    constexpr inline float EPSILON = 1e-6f;

    class PseudoRandom {
        unsigned short seed;
    public:
        PseudoRandom(unsigned short seed) : seed(seed) {}

        PseudoRandom() {
            seed = static_cast<unsigned short>(time(0));
        }

        int rand() {
            seed = (seed + 0x7ed5 + (seed << 6));
            seed = (seed ^ 0xc23c ^ (seed >> 9));
            seed = (seed + 0x1656 + (seed << 3));
            seed = ((seed + 0xa264) ^ (seed << 4));
            seed = (seed + 0xfd70 - (seed << 3));
            seed = (seed ^ 0xba49 ^ (seed >> 8));

            return static_cast<int>(seed);
        }

        void rand(unsigned char* dst, size_t n) {
            for (size_t i = 0; i < n; i++) {
                dst[i] = rand();
            }
        }

        int32_t rand32() {
            return (rand() << 16) | rand();
        }

        uint32_t randU32() {
            return (rand() << 16) | rand();
        }

        int64_t rand64() {
            uint64_t x = randU32();
            uint64_t y = randU32();
            return (x << 32ULL) | y;
        }

        uint64_t randU64() {
            uint64_t x = randU32();
            uint64_t y = randU32();
            return (x << 32ULL) | y;
        }

        float randFloat() {
            return randU32() / static_cast<float>(UINT32_MAX);
        }

        double randDouble() {
            return randU64() / static_cast<double>(UINT64_MAX);
        }

        void setSeed(int number1, int number2) {
            seed = ((static_cast<unsigned short>(number1 * 23729) |
                    static_cast<unsigned short>(number2 % 16786)) ^
                    static_cast<unsigned short>(number2 * number1));
            rand();
        }

        void setSeed(long number) {
            number = shuffle_bits_step(number, 0x2222222222222222ull, 1);
            number = shuffle_bits_step(number, 0x0c0c0c0c0c0c0c0cull, 2);
            number = shuffle_bits_step(number, 0x00f000f000f000f0ull, 4);
            seed = number;
            rand();
        }
    };

    template<typename T>
    inline T sqr(T value) {
        return value * value;
    }

    /// @return integer square of distance between two points
    /// @note glm::distance2 does not support integer vectors
    inline int distance2(const glm::ivec3& a, const glm::ivec3& b) {
        return (b.x - a.x) * (b.x - a.x) +
               (b.y - a.y) * (b.y - a.y) +
               (b.z - a.z) * (b.z - a.z);
    }

    /// @return integer square of distance between two points
    inline int distance2(int ax, int ay, int az, int bx, int by, int bz) {
        return (bx - ax) * (bx - ax) +
               (by - ay) * (by - ay) +
               (bz - az) * (bz - az);
    }

    /// @return integer square of vector length
    /// @note glm::length2 does not support integer vectors
    inline int length2(const glm::ivec3& a) {
        return a.x * a.x + a.y * a.y + a.z * a.z;
    }

    /// @return integer square of vector length
    inline int length2(int x, int y, int z) {
        return x * x + y * y + z * z;
    }

    /// @return integer dot product of two vectors
    inline int dot(const glm::ivec3& a, const glm::ivec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// @brief Find nearest point on segment to given
    /// @param a segment point A
    /// @param b segment point B
    /// @param point given point (may be anywhere)
    /// @return nearest point on the segment to given point 
    inline glm::vec3 closest_point_on_segment(
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& point
    ) {
        auto vec = b - a;
        float da = glm::distance2(point, a);
        float db = glm::distance2(point, b);
        float len = glm::length2(vec);
        float t = (((da - db) / len) * 0.5f + 0.5f);
        t = std::min(1.0f, std::max(0.0f, t));
        return a + vec * t;
    }

    /// @brief Find nearest point on segment to given
    /// @param a segment point A
    /// @param b segment point B
    /// @param point given point (may be anywhere)
    /// @note this overload is actually faster (comment out method to compare)
    /// @return nearest point on the segment to given point 
    inline glm::ivec3 closest_point_on_segment(
        const glm::ivec3& a, const glm::ivec3& b, const glm::ivec3& point
    ) {
        glm::ivec3 vec = b - a;
        int len2 = length2(vec);
    
        if (len2 == 0) return a;
    
        glm::ivec3 ap = point - a;
        int dot_product = dot(ap, vec);
    
        float t = static_cast<float>(dot_product) / static_cast<float>(len2);
        t = glm::clamp(t, 0.0f, 1.0f);
    
        return a + glm::ivec3(glm::round(glm::vec3(vec) * t));
    }
}
