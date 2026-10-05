#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

#include "graphics/core/MeshData.hpp"
#include "maths/fastmaths.hpp"
#include "typedefs.hpp"
#include "world/Environment.hpp"

template <typename VertexStructure>
class Mesh;
class Shader;
class Assets;
class Camera;
class Batch3D;
class Cubemap;
class Framebuffer;
class DrawContext;

struct SkyboxVertex {
    glm::vec2 position;

    static constexpr VertexAttribute ATTRIBUTES[] {
        {VertexAttribute::Type::FLOAT, false, 2}, {{}, 0}};
};

class Skybox {
    SkyMode mode = SkyMode::SOLID;
    uint size;
    std::unique_ptr<Framebuffer> fbo;
    const Assets& assets;
    Shader& shader;
    FastRandom random;
    glm::vec3 lightDir;

    std::unique_ptr<Mesh<SkyboxVertex>> mesh;
    std::unique_ptr<Batch3D> batch3d;
    int frameid = 0;

    float prevMie = -1.0f;
    float prevT = -1.0f;
    float sunAltitude = 45.0f;
    glm::vec3 prevHighlight {1.0f};
    glm::mat4 rotation;

    void drawStars(float angle, float opacity);
    void drawBackground(const Camera& camera, int width, int height);
    void drawSkySprites(
        float daytime,
        float angle,
        float opacity,
        const std::vector<SkySprite>& sprites
    );
    void refreshFace(uint face, Cubemap& cubemap);
public:
    Skybox(uint size, const Assets& assets);
    ~Skybox();

    void setMode(SkyMode mode);

    void draw(
        const Environment& environment,
        const DrawContext& pctx,
        const Camera& camera,
        float daytime,
        float fog
    );

    void refresh(
        const Environment& environment,
        const DrawContext& pctx,
        float t,
        float mie,
        const glm::vec3& tint,
        const glm::vec3& hightlight,
        uint quality
    );

    const Cubemap* getCubemap() const;

    const glm::vec3& getLightDir() const {
        return lightDir;
    }
};
