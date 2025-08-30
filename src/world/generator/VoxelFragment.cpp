#include "VoxelFragment.hpp"

#include <unordered_map>
#include <algorithm>
#include <cstring>

#include "data/dv_util.hpp"
#include "content/Content.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/Block.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/VoxelsVolume.hpp"
#include "voxels/blocks_agent.hpp"
#include "world/Level.hpp"
#include "core_defs.hpp"

std::unique_ptr<VoxelFragment> VoxelFragment::create(
    const Level& level,
    const glm::ivec3& a,
    const glm::ivec3& b,
    bool crop,
    bool entities
) {
    auto start = glm::min(a, b);
    auto size = glm::abs(a - b) + 1;

    if (crop) {
        VoxelsVolume volume(size.x, size.y, size.z);
        volume.setPosition(start.x, start.y, start.z);
        blocks_agent::get_voxels(*level.chunks, &volume);

        auto end = start + size;

        auto min = end;
        auto max = start;
        
        for (int y = start.y; y < end.y; y++) {
            for (int z = start.z; z < end.z; z++) {
                for (int x = start.x; x < end.x; x++) {
                    if (volume.pickBlockId(x, y, z)) {
                        min = glm::min(min, {x, y, z});
                        max = glm::max(max, {x+1, y+1, z+1});
                    }
                }
            }
        }
        if (glm::min(min, max) == min) {
            start = min;
            size = max - min;
        }
    }

    VoxelsVolume volume(size.x, size.y, size.z);
    volume.setPosition(start.x, start.y, start.z);
    blocks_agent::get_voxels(*level.chunks, &volume);

    auto volVoxels = volume.getVoxels();
    std::vector<voxel> voxels(size.x * size.y * size.z);

    std::vector<std::string> blockNames {CORE_AIR};
    std::unordered_map<blockid_t, blockid_t> blocksRegistered {{0, 0}};
    auto contentIndices = level.content.getIndices();
    for (size_t i = 0 ; i < voxels.size(); i++) {
        blockid_t id = volVoxels[i].id;
        blockid_t index;
        
        auto found = blocksRegistered.find(id);
        if (found == blocksRegistered.end()) {
            const auto& def = contentIndices->blocks.require(id);
            index = blockNames.size();
            blockNames.push_back(def.name);
            blocksRegistered[id] = index;
        } else {
            index = found->second;
        }
        voxels[i].id = index;
        voxels[i].state = volVoxels[i].state;
    }

    return std::make_unique<VoxelFragment>(
        size, std::move(voxels), std::move(blockNames));
}

dv::value VoxelFragment::serialize() const {
    auto root = dv::object();
    root["version"] = STRUCTURE_FORMAT_VERSION;
    root["size"] = dv::to_value(size);

    auto& blockNamesArr = root.list("block-names");
    for (const auto& name : blockNames) {
        blockNamesArr.add(name);
    }
    auto& voxelsArr = root.list("voxels");
    for (size_t i = 0; i < voxels.size(); i++) {
        voxelsArr.add(voxels[i].id);
        voxelsArr.add(blockstate2int(voxels[i].state));
    }
    return root;
}

void VoxelFragment::deserialize(const dv::value& src) {
    size = glm::ivec3();
    dv::get_vec(src, "size", size);

    const auto& namesArr = src["block-names"];
    for (const auto& elem : namesArr) {
        blockNames.push_back(elem.asString());
    }

    auto volume = size.x*size.y*size.z;
    voxels.resize(volume);

    const auto& voxelsArr = src["voxels"];
    for (size_t i = 0; i < volume; i++) {
        voxels[i].id = voxelsArr[i * 2].asInteger();
        voxels[i].state = int2blockstate(voxelsArr[i * 2 + 1].asInteger());
    }
}

void VoxelFragment::crop() {
    glm::ivec3 min = size;
    glm::ivec3 max = {};
    
    blockid_t air;
    const auto& found = std::find(blockNames.begin(), blockNames.end(), CORE_AIR);
    if (found == blockNames.end()) {
        throw std::runtime_error(CORE_AIR+" is not found in fragment");
    }
    air = found - blockNames.begin();

    for (int y = 0; y < size.y; y++) {
        for (int z = 0; z < size.z; z++) {
            for (int x = 0; x < size.x; x++) {
                if (voxels[vox_index(x, y, z, size.x, size.z)].id != air) {
                    min = glm::min(min, {x, y, z});
                    max = glm::max(max, {x+1, y+1, z+1});
                }
            }
        }
    }
    if (glm::min(min, max) == min) {
        auto newSize = max - min;
        std::vector<voxel> newVoxels(newSize.x * newSize.y * newSize.z);
        for (int y = 0; y < newSize.y; y++) {
            for (int z = 0; z < newSize.z; z++) {
                for (int x = 0; x < newSize.x; x++) {
                    newVoxels[vox_index(x, y, z, newSize.x, newSize.z)] =
                        voxels[vox_index(
                            x + min.x,
                            y + min.y,
                            z + min.z,
                            size.x,
                            size.z
                        )];
                }
            }
        }
        voxels = std::move(newVoxels);
        size = newSize;
    }
}

void VoxelFragment::prepare(const Content& content) {
    auto volume = size.x*size.y*size.z;
    voxelsRuntime.resize(volume);
    for (size_t i = 0; i < volume; i++) {
        const auto& name = blockNames.at(voxels[i].id);
        voxelsRuntime[i].id = content.blocks.require(name).rt.id;
        voxelsRuntime[i].state = voxels[i].state;
    }
}

void VoxelFragment::place(
    GlobalChunks& chunks, const glm::ivec3& offset
) {
    auto& structVoxels = getRuntimeVoxels();
    for (int y = 0; y < size.y; y++) {
        int sy = y + offset.y;
        if (sy < 0 || sy >= CHUNK_H) {
            continue;
        }
        for (int z = 0; z < size.z; z++) {
            int sz = z + offset.z;
            for (int x = 0; x < size.x; x++) {
                int sx = x + offset.x;
                const auto& structVoxel = 
                    structVoxels[vox_index(x, y, z, size.x, size.z)];
                if (structVoxel.id) {
                    blocks_agent::set(
                        chunks, sx, sy, sz, structVoxel.id, structVoxel.state
                    );
                }
            }
        }
    }
}

std::unique_ptr<VoxelFragment> VoxelFragment::rotated(const Content& content) const {
    std::vector<voxel> newVoxels(voxels.size());

    for (int y = 0; y < size.y; y++) {
        for (int z = 0; z < size.z; z++) {
            for (int x = 0; x < size.x; x++) {
                auto& voxel = newVoxels[vox_index(size.z-z-1, y, x, size.z, size.x)];
                voxel = voxels[vox_index(x, y, z, size.x, size.z)];
                // swap X and Z segment bits
                voxel.state.segment = ((voxel.state.segment & 0b001) << 2)
                                    | (voxel.state.segment & 0b010)
                                    | ((voxel.state.segment & 0b100) >> 2);
                auto& def = content.blocks.require(blockNames[voxel.id]);
                if (def.rotations.name == BlockRotProfile::PANE_NAME ||
                      def.rotations.name == BlockRotProfile::PIPE_NAME) {
                    if (voxel.state.rotation < 4) {
                        voxel.state.rotation = (voxel.state.rotation + 3) & 0b11;
                    }
                } else if (def.rotations.name == BlockRotProfile::STAIRS_NAME) {
                    voxel.state.rotation = ((voxel.state.rotation + 3) & 0b11) |
                                            (voxel.state.rotation & 0b100);
                }
            }
        }
    }
    auto newStructure = std::make_unique<VoxelFragment>(
        // swap X and Z on 90 deg. rotation
        glm::ivec3(size.z, size.y, size.x),
        std::move(newVoxels),
        blockNames
    );
    newStructure->prepare(content);
    return newStructure;
}
