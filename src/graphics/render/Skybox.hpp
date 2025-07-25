#pragma once

#include <memory>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "typedefs.hpp"
#include "maths/fastmaths.hpp"
#include "graphics/core/MeshData.hpp"

template<typename VertexStructure> class Mesh;
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
        {VertexAttribute::Type::FLOAT, false, 2}, 
        {{}, 0}};
};

struct SkySprite {
    std::string texture;
    float phase;
    float distance;
    bool emissive;
    float altitude;
};

class Skybox {
    std::unique_ptr<Framebuffer> fbo;
    uint size;
    Shader& shader;
    bool ready = false;
    FastRandom random;
    glm::vec3 lightDir;

    std::unique_ptr<Mesh<SkyboxVertex>> mesh;
    std::unique_ptr<Batch3D> batch3d;
    std::vector<SkySprite> sprites;
    int frameid = 0;

    float prevMie = -1.0f;
    float prevT = -1.0f;
    float sunAltitude = 45.0f;
    glm::mat4 rotation;

    void drawStars(float angle, float opacity);
    void drawBackground(
        const Camera& camera, const Assets& assets, int width, int height
    );
    void refreshFace(uint face, Cubemap* cubemap);
public:
    Skybox(uint size, Shader& shader);
    ~Skybox();

    void draw(
        const DrawContext& pctx, 
        const Camera& camera, 
        const Assets& assets, 
        float daytime,
        float fog
    );

    void refresh(const DrawContext& pctx, float t, float mie, uint quality);
    void bind() const;
    void unbind() const;
    bool isReady() const {
        return ready;
    }

    const glm::vec3 getLightDir() const {
        return lightDir;
    }
};
