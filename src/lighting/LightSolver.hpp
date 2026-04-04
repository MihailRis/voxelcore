#pragma once

#include "util/array_queue.hpp"

class Chunk;
class Chunks;
class ContentIndices;
class Block;

struct lightentry {
    int x;
    int y;
    int z;
    unsigned char light;
};

class LightSolver {
    util::array_queue<lightentry> addqueue;
    util::array_queue<lightentry> remqueue;
    const Block* const* blockDefs;
    Chunks& chunks;
    int channel;
public:
    LightSolver(const ContentIndices& contentIds, Chunks& chunks, int channel);

    void add(int x, int y, int z);
    void add(int x, int y, int z, int emission);
    void remove(int x, int y, int z);
    void solve(Chunk* prevailingChunk = nullptr);
};
