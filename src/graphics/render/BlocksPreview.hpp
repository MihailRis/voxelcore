#pragma once

#include "typedefs.hpp"

#include <glm/glm.hpp>
#include <memory>

class Assets;
class ImageData;
class Atlas;
class Framebuffer;
class Batch3D;
class Block;
class ContentIndices;
class Shader;
class Window;
class ContentGfxCache;

class BlocksPreview {
    static std::unique_ptr<ImageData> draw(
        const ContentGfxCache& cache,
        Shader& shader,
        const Framebuffer& framebuffer,
        Batch3D& batch,
        const Block& block, 
        int size
    );
public:
    static std::unique_ptr<Atlas> build(
        Window& window,
        const ContentGfxCache& cache,
        const Assets& assets, 
        const ContentIndices& indices
    );
};
