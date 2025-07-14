#pragma once

#include <memory>

#include "typedefs.hpp"

class Level;
class Chunk;
class Chunks;
class Player;
class Lighting;
class WorldGenerator;

/// @brief ChunksController manages chunks dynamic loading/unloading
class ChunksController {
private:
    Level& level;
    std::unique_ptr<WorldGenerator> generator;

    /// @brief Process one chunk: load it or calculate lights for it
    bool loadVisible(const Player& player, uint padding) const;
    bool buildLights(const Player& player, const std::shared_ptr<Chunk>& chunk) const;
    void createChunk(const Player& player, int x, int y) const;
public:
    std::unique_ptr<Lighting> lighting;

    ChunksController(Level& level);
    ~ChunksController();

    /// @param maxDuration milliseconds reserved for chunks loading
    void update(
        int64_t maxDuration, int loadDistance, uint padding, Player& player
    ) const;

    bool isInLoadingZone(const Player& player, uint padding, int x, int z) const;

    const WorldGenerator* getGenerator() const {
        return generator.get();
    }
};
