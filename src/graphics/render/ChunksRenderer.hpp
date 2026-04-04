#pragma once

#include <memory>
#include <vector>
#include <unordered_map>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

#include "util/ThreadPool.hpp"
#include "commons.hpp"

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
    const Chunks& chunks;
    const Assets& assets;
    const Frustum& frustum;
    const EngineSettings& settings;

    std::unique_ptr<BlocksRenderer> renderer;
    std::unordered_map<glm::ivec2, ChunkMesh> meshes;
    std::unordered_map<glm::ivec2, bool> inwork;
    std::vector<ChunksSortEntry> indices;
    util::ThreadPool<RendererJob, RendererResult> threadPool;
    const Mesh<ChunkVertex>* retrieveChunk(
        size_t index, const Camera& camera, bool culling
    );
    std::shared_ptr<VoxelsRenderVolume> prepareVoxelsVolume(const Chunk& chunk);

    size_t enqueuedInFrame = 0;
public:
    ChunksRenderer(
        const Level& level,
        const Chunks& chunks,
        const Assets& assets,
        const Frustum& frustum,
        const ContentGfxCache& cache, 
        const EngineSettings& settings
    );
    virtual ~ChunksRenderer();

    const ChunkMesh* render(
        const std::shared_ptr<Chunk>& chunk, bool important, bool lowPriority
    );
    void unload(const Chunk* chunk);
    void clear();

    const ChunkMesh* getOrRender(
        const std::shared_ptr<Chunk>& chunk, bool important, bool lowPriority
    );

    void drawShadowsPass(
        const Camera& camera, Shader& shader, const Camera& playerCamera
    );

    void drawChunks(const Camera& camera, Shader& shader);

    void drawSortedMeshes(const Camera& camera, Shader& shader);

    void update();

    static size_t visibleChunks;
};
