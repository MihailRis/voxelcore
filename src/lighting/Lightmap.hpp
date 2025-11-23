#pragma once

#include "constants.hpp"
#include "typedefs.hpp"

#include <memory>
#include <cstring>

inline constexpr int LIGHTMAP_DATA_LEN = CHUNK_VOL/2;

// Lichtkarte
class Lightmap {
public:
    light_t map[CHUNK_VOL] {};
    int highestPoint = 0;

    void set(const Lightmap* lightmap);

    void set(const light_t* map);

    void clear() {
        std::memset(map, 0, sizeof(map));
    }

    inline unsigned short get(int x, int y, int z) const {
        return (map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x]);
    }

    inline unsigned char get(int x, int y, int z, int channel) const {
        return (map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x] >> (channel << 2)) & 0xF;
    }

    inline unsigned char getR(int x, int y, int z) const {
        return map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x] & 0xF;
    }

    inline unsigned char getG(int x, int y, int z) const {
        return (map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x] >> 4) & 0xF;
    }

    inline unsigned char getB(int x, int y, int z) const {
        return (map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x] >> 8) & 0xF;
    }

    inline unsigned char getS(int x, int y, int z) const {
        return (map[y*CHUNK_D*CHUNK_W+z*CHUNK_W+x] >> 12) & 0xF;
    }

    inline void setR(int x, int y, int z, int value){
        const int index = y*CHUNK_D*CHUNK_W+z*CHUNK_W+x;
        map[index] = (map[index] & 0xFFF0) | value;
    }

    inline void setG(int x, int y, int z, int value){
        const int index = y*CHUNK_D*CHUNK_W+z*CHUNK_W+x;
        map[index] = (map[index] & 0xFF0F) | (value << 4);
    }

    inline void setB(int x, int y, int z, int value){
        const int index = y*CHUNK_D*CHUNK_W+z*CHUNK_W+x;
        map[index] = (map[index] & 0xF0FF) | (value << 8);
    }

    inline void setS(int x, int y, int z, int value){
        const int index = y*CHUNK_D*CHUNK_W+z*CHUNK_W+x;
        map[index] = (map[index] & 0x0FFF) | (value << 12);
    }

    inline void set(int x, int y, int z, int channel, int value){
        const int index = y*CHUNK_D*CHUNK_W+z*CHUNK_W+x;
        map[index] = (map[index] & (0xFFFF & (~(0xF << (channel*4))))) | (value << (channel << 2));
    }

    inline const light_t* getLights() const {
        return map;
    }

    inline light_t* getLightsWriteable() {
        return map;
    }

    static constexpr light_t combine(int r, int g, int b, int s) {
        return r | (g << 4) | (b << 8) | (s << 12);
    }

    static constexpr light_t extract(light_t light, ubyte channel) {
        return (light >> (channel << 2)) & 0xF;
    }

    std::unique_ptr<ubyte[]> encode() const;
    static std::unique_ptr<light_t[]> decode(const ubyte* buffer);

    static inline light_t SUN_LIGHT_ONLY = combine(0U, 0U, 0U, 15U);
};
