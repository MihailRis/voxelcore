#include "Skybox.hpp"

#include <GL/glew.h>

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>

#include "assets/Assets.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/Cubemap.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Framebuffer.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "maths/UVRegion.hpp"
#include "window/Camera.hpp"
#include "window/Window.hpp"

using namespace advanced_pipeline;

const int STARS_COUNT = 3000;
const int STARS_SEED = 632;

Skybox::Skybox(uint size, const Assets& assets)
    : size(size),
      assets(assets),
      shader(assets.require<Shader>("skybox_gen")),
      batch3d(std::make_unique<Batch3D>(4096)) {

    SkyboxVertex vertices[] {
        {{-1.0f, -1.0f}},
        {{-1.0f, 1.0f}},
        {{1.0f, 1.0f}},
        {{-1.0f, -1.0f}},
        {{1.0f, 1.0f}},
        {{1.0f, -1.0f}}};

    mesh = std::make_unique<Mesh<SkyboxVertex>>(vertices, 6);
    setMode(mode);
}

Skybox::~Skybox() = default;

void Skybox::drawBackground(const Camera& camera, int width, int height) {
    auto backShader = assets.get<Shader>("background");
    backShader->use();
    backShader->uniformMatrix("u_view", camera.getView(false));
    backShader->uniform1f(
        "u_zoom", camera.zoom * camera.getFov() / glm::half_pi<float>()
    );
    backShader->uniform1f("u_ar", float(width) / float(height));
    backShader->uniform1i("u_skybox", 1);
    
    auto texture = fbo->getTexture();
    texture->bind();
    mesh->draw();
    texture->unbind();
}

void Skybox::drawStars(float angle, float opacity) {
    batch3d->texture(nullptr);
    random.setSeed(STARS_SEED);

    glm::mat4 rotation = glm::rotate(
        glm::mat4(1.0f), -angle + glm::pi<float>() * 0.5f, glm::vec3(0, 0, -1)
    );
    rotation = glm::rotate(rotation, sunAltitude, glm::vec3(1, 0, 0));

    float depth = 1e3;
    for (int i = 0; i < STARS_COUNT; i++) {
        float rx = (random.randFloat()) - 0.5f;
        float ry = (random.randFloat()) - 0.5f;
        float rz = (random.randFloat()) - 0.5f;

        glm::vec3 pos = glm::vec4(rx, ry, rz, 1) * rotation;

        float sopacity = random.randFloat();
        if (pos.y < 0.0f) continue;

        sopacity *= (0.2f + std::sqrt(std::cos(angle)) * 0.5f) - 0.05f;
        glm::vec4 tint(1, 1, 1, sopacity * opacity);
        batch3d->point(pos * depth, tint);
    }
    batch3d->flushPoints();
}

void Skybox::drawSkySprites(
    float daytime,
    float angle,
    float opacity,
    const std::vector<SkySprite>& sprites
) {
    float depthScale = 2e3;
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
        glm::vec4 tint(1, 1, 1, opacity);
        if (!sprite.emissive) {
            tint *= 0.6f + std::cos(angle) * 0.4;
        }
        batch3d->sprite(pos, right, up, 1, 1, UVRegion(), tint);
    }
    batch3d->flush();
}

void Skybox::draw(
    const Environment& environment,
    const DrawContext& pctx,
    const Camera& camera,
    float daytime,
    float fog
) {
    const glm::uvec2& viewport = pctx.getViewport();
    drawBackground(camera, viewport.x, viewport.y);

    float angle = daytime * glm::pi<float>() * 2.0f;
    float opacity = glm::pow(1.0f - fog, 7.0f);

    DrawContext ctx = pctx.sub();
    ctx.setBlendMode(BlendMode::addition);

    if (environment.sky.sprites.empty() && !environment.sky.stars) {
        return;
    }

    auto shader = assets.get<Shader>("ui3d");
    shader->use();
    shader->uniformMatrix("u_projview", camera.getProjView(false));
    shader->uniformMatrix("u_apply", glm::mat4(1.0f));
    batch3d->begin();

    if (!environment.sky.sprites.empty()) {
        drawSkySprites(daytime, angle, opacity, environment.sky.sprites);
    }
    if (environment.sky.stars) {
        drawStars(angle, opacity);
    }
}

void Skybox::setMode(SkyMode mode) {
    if (this->mode == mode && fbo) {
        return;
    }
    this->mode = mode;

    uint size = mode == SkyMode::BOX ? this->size : 1U;
    prevT = std::numeric_limits<float>().min();

    uint fboid;
    glGenFramebuffers(1, &fboid);
    fbo = std::make_unique<Framebuffer>(
        fboid,
        false,
        std::make_unique<Cubemap>(size, size, ImageFormat::RGB888)
    );
}

void Skybox::refresh(
    const Environment& environment,
    const DrawContext& pctx,
    float dayTime,
    float mie,
    const glm::vec3& tint,
    const glm::vec3& hightlight,
    uint quality
) {
    setMode(environment.sky.mode);

    if (mode == SkyMode::NONE) {
        return;
    } else if (mode == SkyMode::SOLID) {
        DrawContext ctx = pctx.sub();
        ctx.setDepthMask(false);
        ctx.setDepthTest(false);
        ctx.setFramebuffer(fbo.get());
        ctx.setViewport({fbo->getWidth(), fbo->getHeight()});
        ctx.useTexture(advanced_pipeline::TARGET_SKYBOX, fbo->getTexture());
        ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);

        auto cubemap = dynamic_cast<Cubemap*>(fbo->getTexture());
        assert(cubemap != nullptr);

        for (int face = 0; face < 6; face++) {
            glFramebufferTexture2D(
                GL_FRAMEBUFFER,
                GL_COLOR_ATTACHMENT0,
                GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                cubemap->getId(),
                0
            );
            glClearColor(tint.r, tint.g, tint.b, 1);
            glClear(GL_COLOR_BUFFER_BIT);
        }
        return;
    }

    frameid++;
    DrawContext ctx = pctx.sub();
    ctx.setDepthMask(false);
    ctx.setDepthTest(false);
    ctx.setFramebuffer(fbo.get());
    ctx.setViewport({fbo->getWidth(), fbo->getHeight()});
    ctx.useTexture(advanced_pipeline::TARGET_SKYBOX, fbo->getTexture());
    ctx.useTexture(advanced_pipeline::TARGET_COLOR, nullptr);

    auto cubemap = dynamic_cast<Cubemap*>(fbo->getTexture());
    assert(cubemap != nullptr);

    float t = dayTime * glm::two_pi<float>();
    lightDir = glm::normalize(glm::vec3(sin(t), -cos(t), 0.0f));

    float sunAngle = glm::radians((dayTime - 0.25f) * 360.0f);
    float x = -glm::cos(sunAngle + glm::pi<float>() * 0.5f) *
              glm::radians(sunAltitude);
    float y = sunAngle - glm::pi<float>() * 0.5f;
    float z = glm::radians(0.0f);
    rotation = glm::rotate(glm::mat4(1.0f), y, glm::vec3(0, 1, 0));
    rotation = glm::rotate(rotation, x, glm::vec3(1, 0, 0));
    rotation = glm::rotate(rotation, z, glm::vec3(0, 0, 1));
    lightDir = glm::vec3(rotation * glm::vec4(0, 0, -1, 1));

    shader.use();
    shader.uniform1i("u_quality", quality);
    shader.uniform1f("u_mie", mie);
    shader.uniform3f("u_tint", tint);
    shader.uniform3f("u_hightlight", hightlight);
    shader.uniform1f("u_fog", mie - 1.0f);
    shader.uniform3f("u_lightDir", lightDir);
    shader.uniform1f("u_dayTime", dayTime);

    if (glm::abs(mie - prevMie) + glm::abs(t - prevT) +
            glm::abs(prevHighlight.r - hightlight.r) >=
        0.01) {
        for (uint face = 0; face < 6; face++) {
            refreshFace(face, *cubemap);
        }
    } else {
        uint face = frameid % 6;
        refreshFace(face, *cubemap);
    }
    prevMie = mie;
    prevT = t;
    prevHighlight = hightlight;
}

void Skybox::refreshFace(uint face, Cubemap& cubemap) {
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
        cubemap.getId(),
        0
    );
    shader.uniform3f("u_xaxis", xaxs[face]);
    shader.uniform3f("u_yaxis", yaxs[face]);
    shader.uniform3f("u_zaxis", zaxs[face]);
    mesh->draw();
}

const Cubemap* Skybox::getCubemap() const {
    return dynamic_cast<const Cubemap*>(fbo->getTexture());
}
