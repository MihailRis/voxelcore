#include "ChunksRenderer.hpp"
#include "BlocksRenderer.hpp"
#include "debug/Logger.hpp"
#include "assets/Assets.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/Atlas.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "world/Level.hpp"
#include "window/Camera.hpp"
#include "maths/FrustumCulling.hpp"
#include "maths/util.hpp"
#include "util/listutil.hpp"
#include "util/ObjectsPool.hpp"
#include "util/timeutil.hpp"
#include "settings.hpp"
#include "content/Content.hpp"

static debug::Logger logger("chunks-render");

size_t ChunksRenderer::visibleChunks = 0;

static constexpr inline size_t MAX_CHUNKS_ENQUEUED_IN_FRAME = 4;

class RendererWorker : public util::Worker<RendererJob, RendererResult> {
    BlocksRenderer renderer;
public:
    RendererWorker(
        const Level& level,
        const ContentGfxCache& cache,
        const EngineSettings& settings
    )
        : renderer(
              settings.graphics.denseRender.get()
                  ? settings.graphics.chunkMaxVerticesDense.get()
                  : settings.graphics.chunkMaxVertices.get(),
              level.content.getIndices()->blocks.getDefs(),
              cache,
              settings
          ) {
    }

    RendererResult operator()(const RendererJob& job) override {
        auto chunk = job.chunk;
        auto volume = job.volume;
        renderer.build(chunk.get(), *volume);
        if (renderer.isCancelled()) {
            return RendererResult {
                glm::ivec2(chunk->x, chunk->z), true, ChunkMeshData {}};
        }
        auto meshData = renderer.createMesh();
        return RendererResult {
            glm::ivec2(chunk->x, chunk->z), false, std::move(meshData)};
    }
};

static util::ObjectsPool<VoxelsRenderVolume> voxelsVolumesPool {};

ChunksRenderer::ChunksRenderer(
    const Level& level,
    const Chunks& chunks,
    const Assets& assets,
    const Frustum& frustum,
    const ContentGfxCache& cache,
    const EngineSettings& settings
)
    : chunks(chunks),
      assets(assets),
      frustum(frustum),
      settings(settings),
      threadPool(
          "chunks-render-pool",
          [&]() {
              return std::make_unique<RendererWorker>(
                  level, cache, settings
              );
          },
          [&](RendererResult&& result) {
                if (!result.cancelled) {
                    auto meshData = std::move(result.meshData);
                    auto chunk = std::make_unique<Mesh<ChunkVertex>>(meshData.mesh);
                    meshes[result.key] = ChunkMesh {
                        std::move(chunk),
                        std::move(meshData.sortingMesh),
                        nullptr,
                        std::move(meshData.meshAABB)
                    };
                }
                inwork.erase(result.key);
          },
          settings.graphics.chunkMaxRenderers.get()
      ) {
    threadPool.setStopOnFail(false);
    renderer = std::make_unique<BlocksRenderer>(
        settings.graphics.chunkMaxVertices.get(), 
        level.content.getIndices()->blocks.getDefs(), cache, settings
    );
    logger.info() << "created " << threadPool.getWorkersCount() << " workers";
    logger.info() << "memory consumption is "
                  << renderer->getMemoryConsumption() *
                             threadPool.getWorkersCount() +
                         voxelsVolumesPool.countTotal() *
                             (sizeof(VoxelsVolume) +
                              (CHUNK_W + VOXELS_BUFFER_PADDING * 2) * CHUNK_H *
                                  (CHUNK_D + VOXELS_BUFFER_PADDING * 2) *
                                  (sizeof(voxel) + sizeof(light_t)))
                  << " B";
}

ChunksRenderer::~ChunksRenderer() = default;

std::shared_ptr<VoxelsRenderVolume> ChunksRenderer::prepareVoxelsVolume(
    const Chunk& chunk
) {
    auto voxelsBuffer = voxelsVolumesPool.create();
    voxelsBuffer->setPosition(
        chunk.x * CHUNK_W - VOXELS_BUFFER_PADDING, 0,
        chunk.z * CHUNK_D - VOXELS_BUFFER_PADDING
    );
    chunks.getVoxels(
        *voxelsBuffer, settings.graphics.backlight.get(), chunk.top + 1
    );
    return voxelsBuffer;
}

void ChunksRenderer::renderBlocking(const std::shared_ptr<Chunk>& chunk) {
    glm::ivec2 key(chunk->x, chunk->z);
    ChunkMesh mesh {};
    auto voxelsBuffer = prepareVoxelsVolume(*chunk);
    mesh = renderer->render(chunk.get(), *voxelsBuffer);
    meshes[key] = std::move(mesh);
    chunk->flags.modified = false;
}

void ChunksRenderer::render(
    const std::shared_ptr<Chunk>& chunk, bool lowPriority
) {
    glm::ivec2 key(chunk->x, chunk->z);
    if (inwork.find(key) != inwork.end() ||
        ((inwork.size() >= threadPool.getWorkersCount() ||
          enqueuedInFrame >= MAX_CHUNKS_ENQUEUED_IN_FRAME) &&
         lowPriority)) {
        return;
    }
    chunk->flags.modified = false;
    enqueuedInFrame++;
    auto voxelsBuffer = prepareVoxelsVolume(*chunk);
    threadPool.enqueueJob({chunk, std::move(voxelsBuffer)});
    inwork[key] = true;
}

void ChunksRenderer::unload(const Chunk* chunk) {
    auto found = meshes.find(glm::ivec2(chunk->x, chunk->z));
    if (found != meshes.end()) {
        meshes.erase(found);
    }
}

void ChunksRenderer::clear() {
    meshes.clear();
    inwork.clear();
    threadPool.clearQueue();
}

void ChunksRenderer::update() {
    threadPool.pullResults();
    enqueuedInFrame = 0;

    int width = chunks.getWidth();
    int halfWidth = width / 2;
    int halfHeight = chunks.getHeight() / 2;
    int centerX = chunks.getOffsetX() + halfWidth;
    int centerY = chunks.getOffsetY() + halfHeight;

    meshBuildQueue.clear();
    auto& chunksArray = chunks.getChunks();
    for (int index = 0; index < chunks.getVolume(); index++) {
        const auto& chunk = chunks.getChunks()[index];
        if (chunk == nullptr || !chunk->flags.lighted) {
            continue;
        }
        int x = chunk->x;
        int z = chunk->z;
        if (chunk->flags.modified || meshes.find({x, z}) == meshes.end()) {
            meshBuildQueue.emplace_back(x - centerX, z - centerY);
        }
    }
    std::sort(
        meshBuildQueue.begin(),
        meshBuildQueue.end(),
        [](const glm::ivec2& a, const glm::ivec2& b) {
            return util::length2(a) < util::length2(b);
        }
    );

    int loadDistance = settings.chunks.loadDistance.get();

    const int topN = 10;
    int top = std::min<int>(meshBuildQueue.size(), topN);
    for (int i = 0; i < top; i++) {
        glm::ivec2 offset = meshBuildQueue[i];
        size_t index =
            (offset.y + halfHeight) * width + offset.x + halfWidth;

        auto& chunk = chunksArray[index];
        assert(chunk != nullptr);

        float distance = glm::distance(
            glm::vec3 (centerX, 0.0f, centerY),
            glm::vec3(
                (chunk->x + 0.5f) * CHUNK_W,
                0,
                (chunk->z + 0.5f) * CHUNK_D
            )
        );
        bool important = distance < CHUNK_W * 1.5f;
        bool lowPriority = distance > CHUNK_W * loadDistance * 0.5;

        if (chunk->flags.dirtyHeights) {
            chunk->updateHeights();
        }
        if (important) {
            renderBlocking(chunk);
        } else {
            render(chunk, lowPriority);
        }
    }
}

void ChunksRenderer::drawShadowsPass(
    const Camera& camera, Shader& shader, const Camera& playerCamera
) {
    Frustum frustum;
    frustum.update(camera.getProjView());

    const auto& atlas = assets.require<Atlas>("blocks");

    atlas.getTexture()->bind();

    auto denseDistance = settings.graphics.denseRenderDistance.get();
    auto denseDistance2 = denseDistance * denseDistance;

    for (const auto& chunk : chunks.getChunks()) {
        if (chunk == nullptr) {
            continue;
        }
        glm::ivec2 pos {chunk->x, chunk->z};
        const auto& found = meshes.find(pos);
        if (found == meshes.end()) {
            continue;
        }

        glm::vec3 coord(
            pos.x * CHUNK_W + 0.5f, 0.5f, pos.y * CHUNK_D + 0.5f
        );

        glm::vec3 min(pos.x * CHUNK_W, chunk->bottom, pos.y * CHUNK_D);
        glm::vec3 max(
            pos.x * CHUNK_W + CHUNK_W,
            chunk->top,
            pos.y * CHUNK_D + CHUNK_D
        );

        if (!frustum.isBoxVisible(min, max)) {
            continue;
        }
        glm::mat4 model = glm::translate(glm::mat4(1.0f), coord);
        shader.uniformMatrix("u_model", model);
        found->second.mesh->draw(GL_TRIANGLES, 
            glm::distance2(playerCamera.position * glm::vec3(1, 0, 1), 
                           (min + max) * 0.5f * glm::vec3(1, 0, 1)) < denseDistance2);
    }
}

void ChunksRenderer::drawChunks(
    const Camera& camera, Shader& shader
) {
    const auto& atlas = assets.require<Atlas>("blocks");

    atlas.getTexture()->bind();

    // [warning] this whole method is not thread-safe for chunks

    int chunksWidth = chunks.getWidth();
    int chunksOffsetX = chunks.getOffsetX();
    int chunksOffsetY = chunks.getOffsetY();

    if (indices.size() != chunks.getVolume()) {
        indices.clear();
        for (int i = 0; i < chunks.getVolume(); i++) {
            indices.push_back(ChunksSortEntry {i, 0});
        }
    }
    float px = camera.position.x / static_cast<float>(CHUNK_W) - 0.5f;
    float pz = camera.position.z / static_cast<float>(CHUNK_D) - 0.5f;
    for (auto& index : indices) {
        float x = index.index % chunksWidth + chunksOffsetX - px;
        float z = index.index / chunksWidth + chunksOffsetY - pz;
        index.d = (x * x + z * z) * 1024;
    }
    util::insertion_sort(indices.begin(), indices.end());

    bool culling = settings.graphics.frustumCulling.get();

    visibleChunks = 0;
    shader.uniform1i("u_alphaClip", true);

    auto denseDistance = settings.graphics.denseRenderDistance.get();
    auto denseDistance2 = denseDistance * denseDistance;

    // TODO: minimize draw calls number
    for (int i = indices.size()-1; i >= 0; i--) {
        auto& chunk = chunks.getChunks()[indices[i].index];
        if (chunk == nullptr) {
            continue;
        }
        auto found = meshes.find(glm::ivec2(chunk->x, chunk->z));
        if (found == meshes.end()) {
            continue;
        }
        auto& mesh = found->second;
        if (mesh.mesh == nullptr) {
            continue;
        }
        if (culling) {
            const auto& meshAABB = mesh.meshAABB;
            auto aabbMin = meshAABB.min();
            auto aabbMax = meshAABB.max();
            glm::vec3 min(
                chunk->x * CHUNK_W + std::min(0.0f, aabbMin.x),
                chunk->bottom,
                chunk->z * CHUNK_D + std::min(0.0f, aabbMin.z)
            );
            glm::vec3 max(
                chunk->x * CHUNK_W + aabbMax.x,
                chunk->top,
                chunk->z * CHUNK_D + aabbMax.z
            );

            if (!frustum.isBoxVisible(min, max)) {
                continue;
            }
        }

        glm::vec3 coord(
            chunk->x * CHUNK_W + 0.5f, 0.5f, chunk->z * CHUNK_D + 0.5f
        );
        glm::mat4 model = glm::translate(glm::mat4(1.0f), coord);
        shader.uniformMatrix("u_model", model);
        mesh.mesh->draw(GL_TRIANGLES, glm::distance2(camera.position * glm::vec3(1, 0, 1), 
            (coord + glm::vec3(CHUNK_W * 0.5f, 0.0f, CHUNK_D * 0.5f))) < denseDistance2);
        visibleChunks++;
    }
}

static inline void write_sorting_mesh_entries(
    ChunkVertex* buffer, const std::vector<SortingMeshEntry>& chunkEntries
) {
    for (const auto& entry : chunkEntries) {
        const auto& vertexData = entry.vertexData;
        std::memcpy(
            buffer,
            vertexData.data(),
            vertexData.size() * sizeof(ChunkVertex)
        );
        buffer += vertexData.size();
    }
}

void ChunksRenderer::drawSortedMeshes(const Camera& camera, Shader& shader) {
    const int sortInterval = TRANSLUCENT_BLOCKS_SORT_INTERVAL;
    static int frameid = 0;
    frameid++;

    bool culling = settings.graphics.frustumCulling.get();
    const auto& chunks = this->chunks.getChunks();
    const auto& cameraPos = camera.position;
    const auto& atlas = assets.require<Atlas>("blocks");

    shader.use();
    atlas.getTexture()->bind();
    shader.uniformMatrix("u_model", glm::mat4(1.0f));
    shader.uniform1i("u_alphaClip", false);
    
    for (const auto& index : indices) {
        const auto& chunk = chunks[index.index];
        if (chunk == nullptr || !chunk->flags.lighted) {
            continue;
        }
        const auto& found = meshes.find(glm::ivec2(chunk->x, chunk->z));
        if (found == meshes.end() || found->second.sortingMeshData.entries.empty()) {
            continue;
        }

        if (culling) {
            glm::vec3 min(chunk->x * CHUNK_W, chunk->bottom, chunk->z * CHUNK_D);
            glm::vec3 max(
                chunk->x * CHUNK_W + CHUNK_W,
                chunk->top,
                chunk->z * CHUNK_D + CHUNK_D
            );

            if (!frustum.isBoxVisible(min, max)) continue;
        }

        auto& chunkEntries = found->second.sortingMeshData.entries;

        if (chunkEntries.size() == 1) {
            auto& entry = chunkEntries.at(0);
            if (found->second.sortedMesh == nullptr) {
                found->second.sortedMesh = std::make_unique<Mesh<ChunkVertex>>(
                    entry.vertexData.data(), entry.vertexData.size()
                );
            }
            found->second.sortedMesh->draw();
            continue;
        }
        for (auto& entry : chunkEntries) {
            entry.distance = static_cast<long long>(
                glm::distance2(entry.position, cameraPos)
            );
        }
        if (found->second.sortedMesh == nullptr ||
            (frameid + chunk->x) % sortInterval == 0) {
            std::sort(chunkEntries.begin(), chunkEntries.end());
            size_t size = 0;
            for (const auto& entry : chunkEntries) {
                size += entry.vertexData.size();
            }

            static util::Buffer<ChunkVertex> buffer;
            if (buffer.size() < size) {
                buffer = util::Buffer<ChunkVertex>(size);
            }
            write_sorting_mesh_entries(buffer.data(), chunkEntries);
            found->second.sortedMesh = std::make_unique<Mesh<ChunkVertex>>(
                buffer.data(), size
            );
        }
        found->second.sortedMesh->draw();
    }
}
