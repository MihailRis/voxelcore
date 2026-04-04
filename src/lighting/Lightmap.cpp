#include "Lightmap.hpp"

#include "util/data_io.hpp"

#include <cassert>
#include <cstring>

void Lightmap::set(const Lightmap* lightmap) {
    set(lightmap->map);
}

void Lightmap::set(const light_t* map) {
    std::memcpy(this->map, map, sizeof(light_t) * CHUNK_VOL);
}

static_assert(sizeof(light_t) == 2, "replace dataio calls to new light_t");

std::unique_ptr<ubyte[]> Lightmap::encode() const {
    auto buffer = std::make_unique<ubyte[]>(LIGHTMAP_DATA_LEN);
    for (uint i = 0; i < CHUNK_VOL; i+=2) {
        buffer[i/2] = ((map[i] >> 12) & 0xF) | ((map[i+1] >> 8) & 0xF0);
    }
    return buffer;
}

void Lightmap::decode(const ubyte* src) {
    for (uint i = 0; i < CHUNK_VOL; i+=2) {
        ubyte b = src[i/2];
        map[i] = ((b & 0xF) << 12);
        map[i+1] = ((b & 0xF0) << 8);
    } 
}
