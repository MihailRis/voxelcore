#pragma once

#include "constants.hpp"
#include "typedefs.hpp"
#include "voxel.hpp"

class VoxelsVolume {
    int x, y, z;
    int w, h, d;
    std::unique_ptr<voxel[]> voxels;
    std::unique_ptr<light_t[]> lights;
public:
    VoxelsVolume(int w, int h, int d);
    VoxelsVolume(int x, int y, int z, int w, int h, int d);
    virtual ~VoxelsVolume();

    void setPosition(int x, int y, int z);

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

    voxel* getVoxels() const {
        return voxels.get();
    }

    light_t* getLights() const {
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
};
