#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

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
    struct LoadingState {
        int width = 0;
        int height = 0;
        uint padding = 0;

        int offsetX = 0;
        int offsetZ = 0;
        
        size_t chunksCount = 0;
        std::vector<int> order;
        size_t loadIndex = 0;
        size_t lightIndex = 0;
        bool lightsDirty = true;
        
        float budget = 0.0f;
    };

    Level& level;
    std::unique_ptr<WorldGenerator> generator;
    std::unordered_map<u64id_t, LoadingState> states;

    void syncState(LoadingState& state, Chunks& chunks, uint padding) const;
    void removeFarChunks(Chunks& chunks) const;

    /// @brief Process one chunk: load it or calculate lights for it
    /// @return work units spent, 0 if there is nothing to do
    int loadVisible(
        LoadingState& state, const Player& player, bool isLocalPlayer
    ) const;
    bool buildLights(const Player& player, const std::shared_ptr<Chunk>& chunk) const;
    void createChunk(const Player& player, int x, int y) const;
public:
    std::unique_ptr<Lighting> lighting;

    ChunksController(Level& level);
    ~ChunksController();

    /// @param delta frame time in seconds
    /// @param loadSpeed chunks loading speed
    void update(
        float delta,
        int loadSpeed,
        int loadDistance,
        uint padding,
        Player& player,
        bool isLocalPlayer
    );

    bool isInLoadingZone(const Player& player, uint padding, int x, int z) const;

    const WorldGenerator* getGenerator() const {
        return generator.get();
    }
};
