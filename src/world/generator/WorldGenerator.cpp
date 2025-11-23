#include "WorldGenerator.hpp"

#include <cstring>
#include <algorithm>

#include "maths/util.hpp"
#include "content/Content.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "GeneratorDef.hpp"
#include "VoxelFragment.hpp"
#include "util/timeutil.hpp"
#include "util/listutil.hpp"
#include "maths/voxmaths.hpp"
#include "maths/util.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("world-generator");

/// @brief Max number of biome parameters
static inline constexpr uint MAX_PARAMETERS = 4;

/// @brief Initial + wide_structs + biomes + heightmaps + complete
static inline constexpr uint BASIC_PROTOTYPE_LAYERS = 5;

WorldGenerator::WorldGenerator(
    const GeneratorDef& def, const Content& content, uint64_t seed
)
    : def(def), 
      content(content), 
      seed(seed),
      surroundMap(0, BASIC_PROTOTYPE_LAYERS + def.wideStructsChunksRadius * 2)
{
    def.script->initialize(seed);

    uint levels = BASIC_PROTOTYPE_LAYERS + def.wideStructsChunksRadius * 2;

    surroundMap = SurroundMap(0, levels);
    logger.info() << "total number of prototype levels is " << levels;
    surroundMap.setOutCallback([this](int const x, int const z, int8_t) {
        const auto& found = prototypes.find({x, z});
        if (found == prototypes.end()) {
            logger.warning() << "unable to remove non-existing chunk prototype";
            return;
        }
        prototypes.erase({x, z});
    });
    surroundMap.setLevelCallback(1, [this](int const x, int const z) {
        if (prototypes.find({x, z}) != prototypes.end()) {
            return;
        }
        prototypes[{x, z}] = generatePrototype(x, z);
    });
    surroundMap.setLevelCallback(def.wideStructsChunksRadius + 1, 
    [this](int const x, int const z) {
        generateStructuresWide(requirePrototype(x, z), x, z);
    });
    surroundMap.setLevelCallback(levels-3, [this](int const x, int const z) {
        generateBiomes(requirePrototype(x, z), x, z);
    });
    surroundMap.setLevelCallback(levels-2, [this](int const x, int const z) {
        generateHeightmap(requirePrototype(x, z), x, z);
    });
    surroundMap.setLevelCallback(levels-1, [this](int const x, int const z) {
        generateStructures(requirePrototype(x, z), x, z);
    });
    for (int i = 0; i < def.structures.size(); i++) {
        // pre-calculate rotated structure variants
        def.structures[i]->fragments[0]->prepare(content);
        for (int j = 1; j < 4; j++) {
            def.structures[i]->fragments[j] = 
                def.structures[i]->fragments[j-1]->rotated(content);
        }
    }
}

WorldGenerator::~WorldGenerator() {}

ChunkPrototype& WorldGenerator::requirePrototype(int x, int z) {
    const auto& found = prototypes.find({x, z});
    if (found == prototypes.end()) {
        throw std::runtime_error("prototype not found");
    }
    return *found->second;
}

static inline void generate_pole(
    const BlocksLayers& layers,
    int top, int bottom,
    int seaLevel,
    voxel* voxels,
    int x, int z
) {
    uint y = top;
    uint layerExtension = 0;
    for (const auto& layer : layers.layers) {
        // skip layer if can't be generated under sea level
        if (y < seaLevel && !layer.belowSeaLevel) {
            layerExtension = std::max(0, layer.height);
            continue;
        }
        int layerHeight = layer.height;
        if (layerHeight == -1) {
            // resizeable layer
            layerHeight = y - layers.lastLayersHeight - bottom + 1;
        } else {
            layerHeight += layerExtension;
        }
        layerHeight = std::min(static_cast<uint>(layerHeight), y+1);

        for (uint i = 0; i < layerHeight; i++, y--) {
            voxels[vox_index(x, y, z)].id = layer.rt.id;
        }
        layerExtension = 0;
    }
}

static inline const Biome* choose_biome(
    const std::vector<Biome>& biomes,
    const std::vector<std::shared_ptr<Heightmap>>& maps,
    uint x, uint z
) {
    uint paramsCount = maps.size();
    float params[MAX_PARAMETERS];
    for (uint i = 0; i < paramsCount; i++) {
        params[i] = maps[i]->getUnchecked(x, z);
    }
    const Biome* chosenBiome = nullptr;
    float chosenScore = std::numeric_limits<float>::infinity();
    for (const auto& biome : biomes) {
        float score = 0.0f;
        for (uint i = 0; i < paramsCount; i++) {
            score += glm::abs((params[i] - biome.parameters[i].value) / 
                              biome.parameters[i].weight);
        }
        if (score < chosenScore || std::isinf(chosenScore)) {
            chosenScore = score;
            chosenBiome = &biome;
        }
    }
    assert(chosenBiome != nullptr);
    return chosenBiome;
}

std::unique_ptr<ChunkPrototype> WorldGenerator::generatePrototype(
    int chunkX, int chunkZ
) {
    return std::make_unique<ChunkPrototype>();
}

inline AABB gen_chunk_aabb(int chunkX, int chunkZ) {
    return AABB({chunkX * CHUNK_W, 0, chunkZ * CHUNK_D}, 
                {(chunkX + 1)*CHUNK_W, 256, (chunkZ + 1) * CHUNK_D});
}

void WorldGenerator::placeStructure(
    const StructurePlacement& placement, int priority,
    int chunkX, int chunkZ
) {
    auto& structure =
        *def.structures[placement.structure]->fragments[placement.rotation];
    auto position =
        glm::ivec3(chunkX * CHUNK_W, 0, chunkZ * CHUNK_D) + placement.position;
    auto size = structure.getSize() + glm::ivec3(0, CHUNK_H, 0);
    AABB aabb(position, position + size);
    for (int lcz = -1; lcz <= 1; lcz++) {
        for (int lcx = -1; lcx <= 1; lcx++) {
            const auto& found = prototypes.find({chunkX + lcx, chunkZ + lcz});
            if (found == prototypes.end()) {
                continue;
            }
            auto& otherPrototype = *found->second;
            auto chunkAABB = gen_chunk_aabb(chunkX + lcx, chunkZ + lcz);
            if (chunkAABB.intersect(aabb)) {
                otherPrototype.placements.emplace_back(
                    priority,
                    StructurePlacement {
                        placement.structure,
                        placement.position -
                            glm::ivec3(lcx * CHUNK_W, 0, lcz * CHUNK_D),
                        placement.rotation}
                );
            }
        }
    }
}

void WorldGenerator::placeLine(const LinePlacement& line, int priority) {
    AABB aabb(line.a, line.b);
    aabb.fix();
    aabb.a -= line.radius;
    aabb.b += line.radius;
    int cxa = floordiv<CHUNK_W>(aabb.a.x);
    int cza = floordiv<CHUNK_D>(aabb.a.z);
    int cxb = floordiv<CHUNK_W>(aabb.b.x);
    int czb = floordiv<CHUNK_D>(aabb.b.z);
    for (int cz = cza; cz <= czb; cz++) {
        for (int cx = cxa; cx <= cxb; cx++) {
            const auto& found = prototypes.find({cx, cz});
            if (found != prototypes.end()) {
                found->second->placements.emplace_back(priority, line);
            }
        }
    }
}

void WorldGenerator::placeStructures(
    const std::vector<Placement>& placements, 
    ChunkPrototype& prototype, 
    int chunkX, 
    int chunkZ
) {
    for (const auto& placement : placements) {
        if (auto sp = std::get_if<StructurePlacement>(&placement.placement)) {
            if (sp->structure < 0 || sp->structure >= def.structures.size()) {
                logger.error() << "invalid structure index " << sp->structure;
                continue;
            }
            placeStructure(*sp, placement.priority, chunkX, chunkZ);
        } else {
            const auto& line = std::get<LinePlacement>(placement.placement);
            placeLine(line, placement.priority);
        }
    }
}

void WorldGenerator::generateStructuresWide(
    ChunkPrototype& prototype, int chunkX, int chunkZ
) {
    if (prototype.level >= ChunkPrototypeLevel::WIDE_STRUCTS) {
        return;
    }
    auto placements = def.script->placeStructuresWide(
        {chunkX * CHUNK_W, chunkZ * CHUNK_D}, {CHUNK_W, CHUNK_D}, CHUNK_H
    );
    placeStructures(placements, prototype, chunkX, chunkZ);

    prototype.level = ChunkPrototypeLevel::WIDE_STRUCTS;
}

void WorldGenerator::generateStructures(
    ChunkPrototype& prototype, int chunkX, int chunkZ
) {
    if (prototype.level >= ChunkPrototypeLevel::STRUCTURES) {
        return;
    }
    const auto& biomes = prototype.biomes;
    const auto& heightmap = prototype.heightmap;

    auto placements = def.script->placeStructures(
        {chunkX * CHUNK_W, chunkZ * CHUNK_D}, {CHUNK_W, CHUNK_D},
        heightmap, CHUNK_H
    );
    placeStructures(placements, prototype, chunkX, chunkZ);

    util::PseudoRandom structsRand;
    structsRand.setSeed(chunkX, chunkZ);

    // Place structures defined in biome
    auto heights = heightmap->getValues();
    for (uint z = 0; z < CHUNK_D; z++) {
        for (uint x = 0; x < CHUNK_W; x++) {
            float rand = structsRand.randFloat();
            const Biome* biome = biomes[z * CHUNK_W + x];
            int structureId = biome->structures.choose(rand, -1);
            if (structureId == -1) {
                continue;
            }
            uint8_t rotation = structsRand.randU32() % 4;
            int height = heights[z * CHUNK_W + x] * CHUNK_H;
            if (height < def.seaLevel) {
                continue;
            }
            auto& structure = *def.structures[structureId];
            auto& fragment = *structure.fragments[rotation];
            glm::ivec3 position {x, height-structure.meta.lowering, z};
            position.x -= fragment.getSize().x / 2;
            position.z -= fragment.getSize().z / 2;
            placeStructure(
                StructurePlacement {
                    structureId,
                    position,
                    rotation
                },
                1,
                chunkX, 
                chunkZ
            );
        }
    }
    prototype.level = ChunkPrototypeLevel::STRUCTURES;
}

void WorldGenerator::generateBiomes(
    ChunkPrototype& prototype, int chunkX, int chunkZ
) {
    if (prototype.level >= ChunkPrototypeLevel::BIOMES) {
        return;
    }
    uint bpd = def.biomesBPD;
    auto biomeParams = def.script->generateParameterMaps(
        {floordiv(chunkX * CHUNK_W, bpd), floordiv(chunkZ * CHUNK_D, bpd)},
        {floordiv(CHUNK_W, bpd)+1, floordiv(CHUNK_D, bpd)+1},
        bpd
    );
    for (auto index : def.heightmapInputs) {
        // copy non-scaled maps
        auto copy = std::make_shared<Heightmap>(*biomeParams[index]);
        copy->resize(
            floordiv(CHUNK_W, def.heightsBPD) + 1,
            floordiv(CHUNK_D, def.heightsBPD) + 1,
            def.heightsInterpolation
        );
        prototype.heightmapInputs.push_back(std::move(copy));
    }
    for (const auto& map : biomeParams) {
        map->resize(
            CHUNK_W + bpd, CHUNK_D + bpd, def.biomesInterpolation
        );
        map->crop(0, 0, CHUNK_W, CHUNK_D);
    }
    const auto& biomes = def.biomes;

    auto chunkBiomes = std::make_unique<const Biome*[]>(CHUNK_W*CHUNK_D);
    for (uint z = 0; z < CHUNK_D; z++) {
        for (uint x = 0; x < CHUNK_W; x++) {
            chunkBiomes.get()[z * CHUNK_W + x] =
                choose_biome(biomes, biomeParams, x, z);
        }
    }
    prototype.biomes = std::move(chunkBiomes);
    prototype.level = ChunkPrototypeLevel::BIOMES;
}

void WorldGenerator::generateHeightmap(
    ChunkPrototype& prototype, int chunkX, int chunkZ
) {
    if (prototype.level >= ChunkPrototypeLevel::HEIGHTMAP) {
        return;
    }
    uint bpd = def.heightsBPD;
    prototype.heightmap = def.script->generateHeightmap(
        {floordiv(chunkX * CHUNK_W, bpd), floordiv(chunkZ * CHUNK_D, bpd)},
        {floordiv(CHUNK_W, bpd)+1, floordiv(CHUNK_D, bpd)+1},
        bpd,
        prototype.heightmapInputs
    );
    prototype.heightmap->clamp();
    prototype.heightmap->resize(
        CHUNK_W + bpd, CHUNK_D + bpd, def.heightsInterpolation
    );
    prototype.heightmap->crop(0, 0, CHUNK_W, CHUNK_D);
    prototype.level = ChunkPrototypeLevel::HEIGHTMAP;
}

void WorldGenerator::update(int centerX, int centerY, int loadDistance) {
    surroundMap.setCenter(centerX, centerY);
    surroundMap.resize(loadDistance);
    surroundMap.setCenter(centerX, centerY);
}

void WorldGenerator::generatePlants(
    const ChunkPrototype& prototype,
    float* heights,
    voxel* voxels,
    int chunkX,
    int chunkZ,
    const Biome** biomes
) {
    const auto& indices = content.getIndices()->blocks;
    util::PseudoRandom plantsRand;
    plantsRand.setSeed(chunkX, chunkZ);

    for (uint z = 0; z < CHUNK_D; z++) {
        for (uint x = 0; x < CHUNK_W; x++) {
            const Biome* biome = biomes[z * CHUNK_W + x];

            int height = heights[z * CHUNK_W + x] * CHUNK_H;
            height = std::min(std::max(0, height), CHUNK_H-1);
            
            if (height+1 > def.seaLevel && height+1 < CHUNK_H) {
                float rand = plantsRand.randFloat();
                blockid_t plant = biome->plants.choose(rand);
                if (plant) {
                    auto& voxel = voxels[vox_index(x, height+1, z)];
                    if (voxel.id) {
                        continue;
                    }
                    auto& groundVoxel = voxels[vox_index(x, height, z)];
                    if (indices.get(groundVoxel.id)->rt.solid) {
                        const auto& def = indices.require(plant);
                        voxel = {plant, {}};
                        if (def.rotatable && def.rotations.variantsCount) {
                            voxel.state.rotation =
                                plantsRand.rand() % def.rotations.variantsCount;
                        }
                    }
                }
            }
        }
    }
}

void WorldGenerator::generateLand(
    const ChunkPrototype& prototype,
    float* values,
    voxel* voxels,
    int chunkX,
    int chunkZ,
    const Biome** biomes
) {
    uint seaLevel = def.seaLevel;
    for (uint z = 0; z < CHUNK_D; z++) {
        for (uint x = 0; x < CHUNK_W; x++) {
            const Biome* biome = biomes[z * CHUNK_W + x];

            int height = values[z * CHUNK_W + x] * CHUNK_H;
            height = std::max(0, height);

            const auto& groundLayers = biome->groundLayers;
            const auto& seaLayers = biome->seaLayers;

            generate_pole(seaLayers, seaLevel, height, seaLevel, voxels, x, z);
            generate_pole(groundLayers, height, 0, seaLevel, voxels, x, z);
        }
    }
}

void WorldGenerator::generate(voxel* voxels, int chunkX, int chunkZ) {
    surroundMap.completeAt(chunkX, chunkZ);

    const auto& prototype = requirePrototype(chunkX, chunkZ);
    const auto values = prototype.heightmap->getValues();

    uint seaLevel = def.seaLevel;

    std::memset(voxels, 0, sizeof(voxel) * CHUNK_VOL);

    const auto& biomes = prototype.biomes.get();
    for (uint z = 0; z < CHUNK_D; z++) {
        for (uint x = 0; x < CHUNK_W; x++) {
            const Biome* biome = biomes[z * CHUNK_W + x];

            int height = values[z * CHUNK_W + x] * CHUNK_H;
            height = std::max(0, height);

            const auto& groundLayers = biome->groundLayers;
            const auto& seaLayers = biome->seaLayers;

            generate_pole(seaLayers, seaLevel, height, seaLevel, voxels, x, z);
            generate_pole(groundLayers, height, 0, seaLevel, voxels, x, z);
        }
    }
    generatePlacements(prototype, voxels, chunkX, chunkZ);
    generatePlants(prototype, values, voxels, chunkX, chunkZ, biomes);

    [[maybe_unused]] const auto& indices = content.getIndices()->blocks;
    for (uint i = 0; i < CHUNK_VOL; i++) {
        blockid_t& id = voxels[i].id;
        if (id == BLOCK_STRUCT_AIR) {
            id = BLOCK_AIR;
        }
#ifndef NDEBUG
        if (indices.get(id) == nullptr) {
            abort();
        }
#endif
    }
}

void WorldGenerator::generatePlacements(
    const ChunkPrototype& prototype, voxel* voxels, int chunkX, int chunkZ
) {
    auto placements = prototype.placements;
    std::stable_sort(
        placements.begin(),
        placements.end(), 
        [](const auto& a, const auto& b) {
            return a.priority < b.priority;
        }
    );
    for (const auto& placement : placements) {
        if (auto structure = std::get_if<StructurePlacement>(&placement.placement)) {
            generateStructure(prototype, *structure, voxels, chunkX, chunkZ);
        } else {
            const auto& line = std::get<LinePlacement>(placement.placement);
            generateLine(prototype, line, voxels, chunkX, chunkZ);
        }
    }
}

void WorldGenerator::generateStructure(
    const ChunkPrototype& prototype, 
    const StructurePlacement& placement,
    voxel* voxels, 
    int chunkX, int chunkZ
) {
    if (placement.structure < 0 || placement.structure >= def.structures.size()) {
        logger.error() << "invalid structure index " << placement.structure;
        return;
    }
    auto& generatingStructure = def.structures[placement.structure];
    auto& structure = *generatingStructure->fragments[placement.rotation];
    auto& structVoxels = structure.getRuntimeVoxels();
    const auto& offset = placement.position;
    const auto& size = structure.getSize();

    for (int y = 0; y < size.y; y++) {
        int sy = y + offset.y;
        if (sy < 0 || sy >= CHUNK_H) {
            continue;
        }
        for (int z = 0; z < size.z; z++) {
            int sz = z + offset.z;
            if (sz < 0 || sz >= CHUNK_D) {
                continue;
            }
            for (int x = 0; x < size.x; x++) {
                int sx = x + offset.x;
                if (sx < 0 || sx >= CHUNK_W) {
                    continue;
                }
                const auto& structVoxel = 
                    structVoxels[vox_index(x, y, z, size.x, size.z)];
                if (structVoxel.id) {
                    voxels[vox_index(sx, sy, sz)] = structVoxel;
                }
            }
        }
    }
}

void WorldGenerator::generateLine(
    const ChunkPrototype& prototype, 
    const LinePlacement& line,
    voxel* voxels, 
    int chunkX, int chunkZ
) {
    const auto& indices = content.getIndices()->blocks;

    int cgx = chunkX * CHUNK_W;
    int cgz = chunkZ * CHUNK_D;

    int const radius = line.radius;

    auto a = line.a;
    auto b = line.b;

    int minX = std::max(0, std::min(a.x-radius-cgx, b.x-radius-cgx));
    int maxX = std::min(CHUNK_W, std::max(a.x+radius-cgx, b.x+radius-cgx) + 1);

    int minZ = std::max(0, std::min(a.z-radius-cgz, b.z-radius-cgz));
    int maxZ = std::min(CHUNK_D, std::max(a.z+radius-cgz, b.z+radius-cgz) + 1);

    int minY = std::max(0, std::min(a.y-radius, b.y-radius));
    int maxY = std::min(CHUNK_H, std::max(a.y+radius, b.y+radius) + 1);

    for (int y = minY; y < maxY; y++) {
        for (int z = minZ; z < maxZ; z++) {
            for (int x = minX; x < maxX; x++) {
                int gx = x + cgx;
                int gz = z + cgz;
                glm::ivec3 point {gx, y, gz};
                glm::ivec3 closest = util::closest_point_on_segment(
                    a, b, point
                );
                if (y > 0 &&
                    util::distance2(closest, point) <= radius * radius
                ) {
                    auto& voxel = voxels[vox_index(x, y, z)];
                    if (line.block != BLOCK_AIR) {
                        voxel = {line.block, {}};
                        continue;
                    }
                    if (!indices.require(voxel.id).replaceable) {
                        voxel = {line.block, {}};
                    }
                    auto& below = voxels[vox_index(x, y-1, z)];
                    glm::ivec3 closest2 = util::closest_point_on_segment(
                        a, b, {gx, y-1, gz}
                    );
                    if (util::distance2(closest2, {gx, y-1, gz}) > radius*radius) {
                        const auto& def = indices.require(below.id);
                        if (def.rt.surfaceReplacement != below.id) {
                            below = {def.rt.surfaceReplacement, {}};
                        }
                    }
                }
            }
        }
    }
}

WorldGenDebugInfo WorldGenerator::createDebugInfo() const {
    const auto& area = surroundMap.getArea();
    const auto& levels = area.getBuffer();
    auto values = std::make_unique<ubyte[]>(area.getWidth()*area.getHeight());

    for (uint i = 0; i < levels.size(); i++) {
        values[i] = levels[i];
    }

    return WorldGenDebugInfo {
        area.getOffsetX(),
        area.getOffsetY(),
        static_cast<uint>(area.getWidth()),
        static_cast<uint>(area.getHeight()),
        std::move(values)
    };
}

uint64_t WorldGenerator::getSeed() const {
    return seed;
}
