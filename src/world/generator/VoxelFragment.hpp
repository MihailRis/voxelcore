#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "interfaces/Serializable.hpp"
#include "voxels/voxel.hpp"

inline constexpr int STRUCTURE_FORMAT_VERSION = 1;

class Level;
class Content;
class GlobalChunks;

class VoxelFragment : public Serializable {
    glm::ivec3 size;

    /// @brief Structure voxels indexed different to world content
    std::vector<voxel> voxels;
    /// @brief Block names are used for indexing
    std::vector<std::string> blockNames;

    /// @brief Structure voxels built on prepare(...) call
    std::vector<voxel> voxelsRuntime;
public:
    VoxelFragment() : size() {}

    VoxelFragment(
        glm::ivec3 size,
        std::vector<voxel> voxels,
        std::vector<std::string> blockNames
    ):  size(size), 
        voxels(std::move(voxels)), 
        blockNames(std::move(blockNames)) 
    {}

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
    void crop();

    /// @brief Build runtime voxel indices
    /// @param content world content
    void prepare(const Content& content);

    /// @brief Place fragment to the world
    /// @param offset target location
    void place(GlobalChunks& chunks, const glm::ivec3& offset);

    /// @brief Create structure copy rotated 90 deg. clockwise
    std::unique_ptr<VoxelFragment> rotated(const Content& content) const;

    static std::unique_ptr<VoxelFragment> create(
        const Level& level,
        const glm::ivec3& a,
        const glm::ivec3& b,
        bool crop,
        bool entities
    );

    const glm::ivec3& getSize() const {
        return size;
    }

    /// @return Voxels with indices valid to current world content
    const std::vector<voxel>& getRuntimeVoxels() {
        assert(!voxelsRuntime.empty());
        return voxelsRuntime;
    }
};
