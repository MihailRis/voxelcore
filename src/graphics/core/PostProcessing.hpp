#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include "MeshData.hpp"

template<typename VertexStructure> class Mesh;
class Assets;
class Framebuffer;
class DrawContext;
class ImageData;
class PostEffect;
class Camera;

struct PostProcessingVertex {
    glm::vec2 position;

    static constexpr VertexAttribute ATTRIBUTES[] {
        {VertexAttribute::Type::FLOAT, false, 2}, 
        {{}, 0}};
};

/// @brief Framebuffer with blitting with shaders.
/// @attention Current implementation does not support multiple render passes 
/// for multiple effects. Will be implemented in v0.21
class PostProcessing {
    /// @brief Main framebuffer (lasy field)
    std::unique_ptr<Framebuffer> fbo;
    std::unique_ptr<Framebuffer> fboSecond;
    /// @brief Fullscreen quad mesh as the post-processing canvas
    std::unique_ptr<Mesh<PostProcessingVertex>> quadMesh;
    std::vector<std::shared_ptr<PostEffect>> effectSlots;

    std::vector<glm::vec3> ssaoKernel;
    uint noiseTexture;
    bool ssaoConfigured = false;
public:
    PostProcessing(size_t effectSlotsCount);
    ~PostProcessing();

    /// @brief Prepare and bind framebuffer
    /// @param context graphics context will be modified
    void use(DrawContext& context);

    /// @brief Render fullscreen quad using the passed shader 
    /// with framebuffer texture bound
    /// @param context graphics context
    /// @throws std::runtime_error if use(...) wasn't called before
    void render(const DrawContext& context, const Assets& assets, float timer, const Camera& camera, uint depthMap);

    void setEffect(size_t slot, std::shared_ptr<PostEffect> effect);

    PostEffect* getEffect(size_t slot);

    /// @brief Make an image from the last rendered frame
    std::unique_ptr<ImageData> toImage();

    Framebuffer* getFramebuffer() const;
};
