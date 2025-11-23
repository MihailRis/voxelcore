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

class ChunksRenderer {
    const Chunks& chunks;
    const Assets& assets;
    const Frustum& frustum;
    const EngineSettings& settings;

    std::unique_ptr<BlocksRenderer> renderer;
    std::unordered_map<glm::ivec2, ChunkMesh> meshes;
    std::unordered_map<glm::ivec2, bool> inwork;
    std::vector<ChunksSortEntry> indices;
    util::ThreadPool<std::shared_ptr<Chunk>, RendererResult> threadPool;
    const Mesh<ChunkVertex>* retrieveChunk(
        size_t index, const Camera& camera, bool culling
    );
public:
    ChunksRenderer(
        const Level* level,
        const Chunks& chunks,
        const Assets& assets,
        const Frustum& frustum,
        const ContentGfxCache& cache, 
        const EngineSettings& settings
    );
    virtual ~ChunksRenderer();

    const Mesh<ChunkVertex>* render(
        const std::shared_ptr<Chunk>& chunk, bool important
    );
    void unload(const Chunk* chunk);
    void clear();

    const Mesh<ChunkVertex>* getOrRender(
        const std::shared_ptr<Chunk>& chunk, bool important
    );

    void drawShadowsPass(
        const Camera& camera, Shader& shader, const Camera& playerCamera
    );

    void drawChunks(const Camera& camera, Shader& shader);

    void drawSortedMeshes(const Camera& camera, Shader& shader);

    void update();

    static size_t visibleChunks;
};
