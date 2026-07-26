#pragma once

#include "typedefs.hpp"

class ContentIndices;
class Chunk;
class Chunks;
class LightSolver;

class Lighting {
    const ContentIndices& indices;
    Chunks& chunks;
    std::unique_ptr<LightSolver> solverR;
    std::unique_ptr<LightSolver> solverG;
    std::unique_ptr<LightSolver> solverB;
    std::unique_ptr<LightSolver> solverS;
public:
    Lighting(const ContentIndices& indices, Chunks& chunks);
    ~Lighting();

    void clear();
    void buildSkyLight(int cx, int cz);
    void onChunkLoaded(int cx, int cz, bool expand);
    void onBlockSet(int x, int y, int z, blockid_t id);

    static void prebuildSkyLight(Chunk& chunk, const ContentIndices& indices);
};
