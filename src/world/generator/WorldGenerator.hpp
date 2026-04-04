#pragma once

#include <array>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "constants.hpp"
#include "typedefs.hpp"
#include "voxels/voxel.hpp"
#include "SurroundMap.hpp"
#include "StructurePlacement.hpp"

class Content;
struct GeneratorDef;
class Heightmap;
struct Biome;
class VoxelFragment;

enum class ChunkPrototypeLevel {
    VOID=0, WIDE_STRUCTS, BIOMES, HEIGHTMAP, STRUCTURES
};

struct ChunkPrototype {
    ChunkPrototypeLevel level = ChunkPrototypeLevel::VOID;

    /// @brief chunk biomes matrix
    std::unique_ptr<const Biome*[]> biomes;

    /// @brief chunk heightmap
    std::shared_ptr<Heightmap> heightmap;

    std::vector<Placement> placements;

    /// @brief biome parameters maps saved until heightmaps generation
    std::vector<std::shared_ptr<Heightmap>> heightmapInputs {};
};

struct WorldGenDebugInfo {
    int areaOffsetX;
    int areaOffsetY;
    uint areaWidth;
    uint areaHeight;
    std::unique_ptr<ubyte[]> areaLevels;
};

/// @brief High-level world generation controller
class WorldGenerator {
    /// @param def generator definition
    const GeneratorDef& def;
    /// @param content world content
    const Content& content;
    /// @param seed world seed
    uint64_t seed;
    /// @brief Chunk prototypes main storage
    std::unordered_map<glm::ivec2, std::unique_ptr<ChunkPrototype>> prototypes;
    /// @brief Chunk prototypes loading surround map
    SurroundMap surroundMap;

    /// @brief Generate chunk prototype (see ChunkPrototype)
    /// @param x chunk position X divided by CHUNK_W
    /// @param z chunk position Y divided by CHUNK_D
    std::unique_ptr<ChunkPrototype> generatePrototype(int x, int z);

    ChunkPrototype& requirePrototype(int x, int z);

    void generateStructuresWide(ChunkPrototype& prototype, int x, int z);

    void generateStructures(ChunkPrototype& prototype, int x, int z);

    void generateBiomes(ChunkPrototype& prototype, int x, int z);

    void generateHeightmap(ChunkPrototype& prototype, int x, int z);

    void placeStructure(
        const StructurePlacement& placement, int priority, 
        int chunkX, int chunkZ
    );

    void placeLine(const LinePlacement& line, int priority);
    void placeBlock(const BlockPlacement& block, int priority);

    void generatePlacements(
        const ChunkPrototype& prototype, voxel* voxels, int x, int z
    );
    void generateLine(
        const ChunkPrototype& prototype, 
        const LinePlacement& placement,
        voxel* voxels, 
        int x, int z
    );
    void generateBlock(
        const ChunkPrototype& prototype,
        const BlockPlacement& placement,
        voxel* voxels,
        int x, int z
    );
    void generateStructure(
        const ChunkPrototype& prototype, 
        const StructurePlacement& placement,
        voxel* voxels, 
        int x, int z
    );
    void generatePlants(
        const ChunkPrototype& prototype,
        float* values,
        voxel* voxels,
        int x,
        int z,
        const Biome** biomes
    );
    void generateLand(
        const ChunkPrototype& prototype,
        float* values,
        voxel* voxels,
        int x,
        int z,
        const Biome** biomes
    );

    void placeStructures(
        const std::vector<Placement>& placements,
        ChunkPrototype& prototype,
        int x, int z
    );
public:
    WorldGenerator(
        const GeneratorDef& def,
        const Content& content,
        uint64_t seed
    );
    ~WorldGenerator();

    void update(int centerX, int centerY, int loadDistance);

    /// @brief Generate complete chunk voxels
    /// @param voxels destinatiopn chunk voxels buffer
    /// @param x chunk position X divided by CHUNK_W
    /// @param z chunk position Y divided by CHUNK_D
    void generate(voxel* voxels, int x, int z);

    WorldGenDebugInfo createDebugInfo() const;

    uint64_t getSeed() const;
};
