#pragma once

#include "constants.hpp"
#include "typedefs.hpp"
#include "voxel.hpp"

#include <cstring>

class Content;

class VoxelsVolume {
public:
    VoxelsVolume(int w, int h, int d)
     : VoxelsVolume(0, 0, 0, w, h, d) {}

    VoxelsVolume(int x, int y, int z, int w, int h, int d)
        : x(x), y(y), z(z), w(w), h(h), d(d),
          voxels(std::make_unique<voxel[]>(w * h * d)),
          lights(std::make_unique<light_t[]>(w * h * d)) {
        std::memset(voxels.get(), 0xFF, sizeof(voxel) * w * h * d);
    }

    void setPosition(int x, int y, int z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    int getX() const {
        return x;
    }

    int getY() const {
        return y;
    }

    int getZ() const {
        return z;
    }

    int getW() const {
        return w;
    }

    int getH() const {
        return h;
    }

    int getD() const {
        return d;
    }

    voxel* getVoxels() {
        return voxels.get();
    }

    const voxel* getVoxels() const {
        return voxels.get();
    }

    light_t* getLights() {
        return lights.get();
    }

    const light_t* getLights() const {
        return lights.get();
    }

    inline blockid_t pickBlockId(int bx, int by, int bz) const {
        if (bx < x || by < y || bz < z || bx >= x + w || by >= y + h ||
            bz >= z + d) {
            return BLOCK_VOID;
        }
        return voxels[vox_index(bx - x, by - y, bz - z, w, d)].id;
    }

    
    inline voxel pickBlock(int bx, int by, int bz) const {
        if (bx < x || by < y || bz < z || bx >= x + w || by >= y + h ||
            bz >= z + d) {
            return {BLOCK_VOID, {}};
        }
        return voxels[vox_index(bx - x, by - y, bz - z, w, d)];
    }

    inline light_t pickLight(int bx, int by, int bz) const {
        if (bx < x || by < y || bz < z || bx >= x + w || by >= y + h ||
            bz >= z + d) {
            return 0;
        }
        return lights[vox_index(bx - x, by - y, bz - z, w, d)];
    }

    void compressInto(VoxelsVolume& dst, const Content& content) const;
private:
    int x, y, z;
    int w, h, d;
    std::unique_ptr<voxel[]> voxels;
    std::unique_ptr<light_t[]> lights;
};


template <int w, int h, int d>
class StaticVoxelsVolume {
public:
    static inline constexpr size_t size = w * h * d;
    static inline constexpr int width = w;
    static inline constexpr int height = h;
    static inline constexpr int depth = d;

    StaticVoxelsVolume()
     : StaticVoxelsVolume(0, 0, 0) {}
    
    StaticVoxelsVolume(int x, int y, int z)
    : x(x), y(y), z(z) {
        std::memset(voxels, 0xFF, sizeof(voxel) * size);
    }

    void setPosition(int x, int y, int z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    int getX() const {
        return x;
    }

    int getY() const {
        return y;
    }

    int getZ() const {
        return z;
    }

    voxel* getVoxels() {
        return voxels;
    }

    light_t* getLights() {
        return lights;
    }

    inline blockid_t pickBlockId(uint bx, uint by, uint bz) const {
        bx -= x;
        by -= y;
        bz -= z;
        if (bx >= w || by >= h || bz >= d) {
            return BLOCK_VOID;
        }
        return voxels[vox_index(bx, by, bz, w, d)].id;
    }

    
    inline const voxel& pickBlock(uint bx, uint by, uint bz) const {
        bx -= x;
        by -= y;
        bz -= z;
        if (bx >= w || by >= h || bz >= d) {
            static voxel voidVoxel {BLOCK_VOID, {}};
            return voidVoxel;
        }
        return voxels[vox_index(bx, by, bz, w, d)];
    }

    inline light_t pickLight(uint bx, uint by, uint bz) const {
        bx -= x;
        by -= y;
        bz -= z;
        if (bx >= w || by >= h || bz >= d) {
            return 0;
        }
        return lights[vox_index(bx, by, bz, w, d)];
    }
private:
    int x, y, z;
    voxel voxels[size];
    light_t lights[size];
};

