#include "Skybox.hpp"
#include "assets/Assets.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/Cubemap.hpp"
#include "graphics/core/Framebuffer.hpp"
#include "graphics/core/DrawContext.hpp"
#include "window/Window.hpp"
#include "window/Camera.hpp"
#include "maths/UVRegion.hpp"

#include <cmath>
#include <iostream>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>

using namespace advanced_pipeline;

const int STARS_COUNT = 3000;
const int STARS_SEED = 632;

Skybox::Skybox(uint size, Shader& shader)
  : size(size),
    shader(shader),
    batch3d(std::make_unique<Batch3D>(4096))
{
    auto cubemap = std::make_unique<Cubemap>(size, size, ImageFormat::rgb888);

    uint fboid;
    glGenFramebuffers(1, &fboid);
    fbo = std::make_unique<Framebuffer>(fboid, 0, std::move(cubemap));

    SkyboxVertex vertices[]{
        {{-1.0f, -1.0f}},
        {{-1.0f, 1.0f}},
        {{1.0f, 1.0f}},
        {{-1.0f, -1.0f}},
        {{1.0f, 1.0f}},
        {{1.0f, -1.0f}}
    };

    mesh = std::make_unique<Mesh<SkyboxVertex>>(vertices, 6);

    sprites.push_back(SkySprite {
        "misc/moon",
        glm::pi<float>() * 0.5f,
        4.0f,
        false,
        glm::pi<float>() * 0.25f,
    });
    
    sprites.push_back(SkySprite {
        "misc/moon_flare",
        glm::pi<float>() * 0.5f,
        0.5f,
        false,
        glm::pi<float>() * 0.25f,
    });

    sprites.push_back(SkySprite {
        "misc/sun",
        glm::pi<float>() * 1.5f,
        4.0f,
        true,
        glm::pi<float>() * 0.25f,
    });
}

Skybox::~Skybox() = default;

void Skybox::drawBackground(
    const Camera& camera, const Assets& assets, int width, int height
) {
    auto backShader = assets.get<Shader>("background");
    backShader->use();
    backShader->uniformMatrix("u_view", camera.getView(false));
    backShader->uniform1f(
        "u_zoom", camera.zoom * camera.getFov() / glm::half_pi<float>()
    );
    backShader->uniform1f("u_ar", float(width)/float(height));
    backShader->uniform1i("u_skybox", 1);
    bind();
    mesh->draw();
    unbind();
}

void Skybox::drawStars(float angle, float opacity) {
    batch3d->texture(nullptr);
    random.setSeed(STARS_SEED);

    glm::mat4 rotation = glm::rotate(
        glm::mat4(1.0f),
        -angle + glm::pi<float>() * 0.5f,
        glm::vec3(0, 0, -1)
    );
    rotation = glm::rotate(rotation, sunAltitude, glm::vec3(1, 0, 0));

    float depth = 1e3;
    for (int i = 0; i < STARS_COUNT; i++) {
        float rx = (random.randFloat()) - 0.5f;
        float ry = (random.randFloat()) - 0.5f;
        float rz = (random.randFloat()) - 0.5f;

        glm::vec3 pos = glm::vec4(rx, ry, rz, 1) * rotation;

        float sopacity = random.randFloat();
        if (pos.y < 0.0f)
            continue;

        sopacity *= (0.2f + std::sqrt(std::cos(angle)) * 0.5f) - 0.05f;
        glm::vec4 tint (1,1,1, sopacity * opacity);
        batch3d->point(pos * depth, tint);
    }
    batch3d->flushPoints();
}

void Skybox::draw(
    const DrawContext& pctx,
    const Camera& camera,
    const Assets& assets,
    float daytime,
    float fog)
{
    const glm::uvec2& viewport = pctx.getViewport();

    glActiveTexture(GL_TEXTURE0);

    drawBackground(camera, assets, viewport.x, viewport.y);

    DrawContext ctx = pctx.sub();
    ctx.setBlendMode(BlendMode::addition);

    auto p_shader = assets.get<Shader>("ui3d");
    p_shader->use();
    p_shader->uniformMatrix("u_projview", camera.getProjView(false));
    p_shader->uniformMatrix("u_apply", glm::mat4(1.0f));
    batch3d->begin();

    float angle = daytime * glm::pi<float>() * 2.0f;
    float opacity = glm::pow(1.0f - fog, 7.0f);

    float depthScale = 1e3;
    for (auto& sprite : sprites) {
        batch3d->texture(assets.get<Texture>(sprite.texture));

        float sangle = daytime * glm::pi<float>() * 2.0 + sprite.phase;
        float distance = sprite.distance * depthScale;

        glm::mat4 rotation = glm::rotate(
            glm::mat4(1.0f),
            -sangle + glm::pi<float>() * 0.5f,
            glm::vec3(0, 0, -1)
        );
        rotation = glm::rotate(rotation, sprite.altitude, glm::vec3(1, 0, 0));
        glm::vec3 pos = glm::vec4(0, distance, 0, 1) * rotation;
        glm::vec3 up = glm::vec4(depthScale, 0, 0, 1) * rotation;
        glm::vec3 right = glm::vec4(0, 0, depthScale, 1) * rotation;
        glm::vec4 tint (1,1,1, opacity);
        if (!sprite.emissive) {
            tint *= 0.6f + std::cos(angle)*0.4;
        }
        batch3d->sprite(pos, right,
                        up, 1, 1, UVRegion(), tint);
    }
    batch3d->flush();
    drawStars(angle, opacity);
}

void Skybox::refresh(const DrawContext& pctx, float t, float mie, uint quality) {
    frameid++;
    float dayTime = t;
    DrawContext ctx = pctx.sub();
    ctx.setDepthMask(false);
    ctx.setDepthTest(false);
    ctx.setFramebuffer(fbo.get());
    ctx.setViewport({size, size});

    auto cubemap = dynamic_cast<Cubemap*>(fbo->getTexture());
    assert(cubemap != nullptr);

    ready = true;
    glActiveTexture(GL_TEXTURE0 + TARGET_SKYBOX);
    cubemap->bind();
    shader.use();
    t *= glm::two_pi<float>();

    lightDir = glm::normalize(glm::vec3(sin(t), -cos(t), 0.0f));

    float sunAngle = glm::radians((t / glm::two_pi<float>() - 0.25f) * 360.0f);
    float x = -glm::cos(sunAngle + glm::pi<float>() * 0.5f) * glm::radians(sunAltitude);
    float y = sunAngle - glm::pi<float>() * 0.5f;
    float z = glm::radians(0.0f);
    rotation = glm::rotate(glm::mat4(1.0f), y, glm::vec3(0, 1, 0));
    rotation = glm::rotate(rotation, x, glm::vec3(1, 0, 0));
    rotation = glm::rotate(rotation, z, glm::vec3(0, 0, 1));
    lightDir = glm::vec3(rotation * glm::vec4(0, 0, -1, 1));

    shader.uniform1i("u_quality", quality);
    shader.uniform1f("u_mie", mie);
    shader.uniform1f("u_fog", mie - 1.0f);
    shader.uniform3f("u_lightDir", lightDir);
    shader.uniform1f("u_dayTime", dayTime);

    if (glm::abs(mie-prevMie) + glm::abs(t-prevT) >= 0.01) {
        for (uint face = 0; face < 6; face++) {
            refreshFace(face, cubemap);
        }
    } else {
        uint face = frameid % 6;
        refreshFace(face, cubemap);
    }
    prevMie = mie;
    prevT = t;

    cubemap->unbind();
    glActiveTexture(GL_TEXTURE0);
}

void Skybox::refreshFace(uint face, Cubemap* cubemap) {
    const glm::vec3 xaxs[] = {
        {0.0f, 0.0f, -1.0f},
        {0.0f, 0.0f, 1.0f},
        {-1.0f, 0.0f, 0.0f},

        {-1.0f, 0.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    };
    const glm::vec3 yaxs[] = {
        {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, -1.0f},

        {0.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
    };

    const glm::vec3 zaxs[] = {
        {1.0f, 0.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f},
        {0.0f, -1.0f, 0.0f},

        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, -1.0f},
        {0.0f, 0.0f, 1.0f},
    };
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
        cubemap->getId(),
        0
    );
    shader.uniform3f("u_xaxis", xaxs[face]);
    shader.uniform3f("u_yaxis", yaxs[face]);
    shader.uniform3f("u_zaxis", zaxs[face]);
    mesh->draw();
}

void Skybox::bind() const {
    glActiveTexture(GL_TEXTURE0 + TARGET_SKYBOX);
    fbo->getTexture()->bind();
    glActiveTexture(GL_TEXTURE0);
}

void Skybox::unbind() const {
    glActiveTexture(GL_TEXTURE0 + TARGET_SKYBOX);
    fbo->getTexture()->unbind();
    glActiveTexture(GL_TEXTURE0);
}
