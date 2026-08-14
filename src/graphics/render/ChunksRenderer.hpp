#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include "util/ThreadPool.hpp"
#include "commons.hpp"

#include <memory>
#include <vector>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

template<typename VertexStructure> class Mesh;
class Chunk;
class Level;
class Camera;
class Shader;
class Assets;
class Chunks;
class Frustum;
class BlocksRenderer;
class ContentGfxCache;
struct EngineSettings;

struct ChunksSortEntry {
    int index;
    int d;

    inline bool operator<(const ChunksSortEntry& o) const noexcept {
        return d > o.d;
    }
};

struct RendererResult {
    glm::ivec2 key;
    bool cancelled;
    ChunkMeshData meshData;
};

struct RendererJob {
    std::shared_ptr<Chunk> chunk;
    std::shared_ptr<VoxelsRenderVolume> volume;
};

class ChunksRenderer {
public:
    ChunksRenderer(
        const Level& level,
        const Chunks& chunks,
        const Assets& assets,
        const Frustum& frustum,
        const ContentGfxCache& cache, 
        const EngineSettings& settings
    );
    ~ChunksRenderer();

    void unload(const Chunk* chunk);
    void clear();

    void drawShadowsPass(
        const Camera& camera, Shader& shader, const Camera& playerCamera
    );

    void drawChunks(const Camera& camera, Shader& shader);

    void drawSortedMeshes(const Camera& camera, Shader& shader);

    void update();

    static size_t visibleChunks;

private:
    const Chunks& chunks;
    const Assets& assets;
    const Frustum& frustum;
    const EngineSettings& settings;

    std::unique_ptr<BlocksRenderer> renderer;
    std::unordered_map<glm::ivec2, ChunkMesh> meshes;
    std::unordered_map<glm::ivec2, bool> inwork;
    std::vector<ChunksSortEntry> indices;
    util::ThreadPool<RendererJob, RendererResult> threadPool;
    std::vector<glm::ivec2> meshBuildQueue;

    size_t enqueuedInFrame = 0;

    std::shared_ptr<VoxelsRenderVolume> prepareVoxelsVolume(const Chunk& chunk);

    void render(const std::shared_ptr<Chunk>& chunk, bool lowPriority);

    void renderBlocking(const std::shared_ptr<Chunk>& chunk);
};
