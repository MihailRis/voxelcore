#include "VoxelsVolume.hpp"

#include "constants.hpp"
#include "content/Content.hpp"
#include "voxels/Block.hpp"

#include <assert.h>

void VoxelsVolume::compressInto(VoxelsVolume& dst, const Content& content) const {
    assert(dst.w < w && dst.h < h && dst.d < d);
    assert(w % dst.w == 0 && h % dst.h == 0 && d % dst.d == 0);

    int stepW = w / dst.w;
    int stepH = h / dst.h;
    int stepD = d / dst.d;
    int height = dst.h;
    int depth = dst.d;
    int width = dst.w;

    auto dstVoxels = dst.getVoxels();
    auto dstLights = dst.getLights();

    const auto& blockDefs = content.getIndices()->blocks;

    for (int y = 0; y < height; y++) {
        for (int z = 0; z < depth; z++) {
            for (int x = 0; x < width; x++) {
                voxel selectedVoxel {BLOCK_AIR, {}};
                light_t light = 0;
                for (int ly = 0; ly < stepH; ly++) {
                    for (int lz = 0; lz < stepD; lz++) {
                        for (int lx = 0; lx < stepW; lx++) {
                            size_t srcIndex = vox_index(
                                x * stepW + lx, y * stepH + ly, z * stepD, w, d
                            );
                            auto vox = voxels[srcIndex];
                            if (vox.id == BLOCK_VOID) {
                                continue;
                            }
                            const auto& def = blockDefs.require(vox.id);
                            if (def.rt.solid) {
                                selectedVoxel = std::move(vox);
                            } else if (light == 0) {
                                light = lights[srcIndex];
                            }
                        }
                    }
                }
                size_t dstIndex = vox_index(x, y, z, dst.w, dst.d);
                dstLights[dstIndex] = light;
                dstVoxels[dstIndex] = std::move(selectedVoxel);
            }
        }
    }
}
