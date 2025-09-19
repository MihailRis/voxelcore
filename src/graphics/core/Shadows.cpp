#include "Shadows.hpp"

#include <GL/glew.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include "assets/Assets.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/commons.hpp"
#include "world/Level.hpp"
#include "world/Weather.hpp"
#include "world/World.hpp"

using namespace advanced_pipeline;

inline constexpr int MIN_SHADOW_MAP_RES = 512;
inline constexpr GLenum TEXTURE_MAIN = GL_TEXTURE0;

class ShadowMap {
public:
    ShadowMap(int resolution) : resolution(resolution) {
        glGenTextures(1, &depthMap);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
            resolution, resolution, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
        glTexParameteri(
            GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE
        );
        float border[4] {1.0f, 1.0f, 1.0f, 1.0f};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(
            GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0
        );
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ~ShadowMap() {
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &depthMap);
    }

    void bind(){
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    void unbind() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    uint getDepthMap() const {
        return depthMap;
    }
    int getResolution() const {
        return resolution;
    }
private:
    uint fbo;
    uint depthMap; 
    int resolution;
};

Shadows::Shadows(const Level& level) : level(level) {}

Shadows::~Shadows() = default;

void Shadows::setQuality(int quality) {
    int resolution = MIN_SHADOW_MAP_RES << quality;
    if (quality > 0 && !shadows) {
        shadowMap = std::make_unique<ShadowMap>(resolution);
        wideShadowMap = std::make_unique<ShadowMap>(resolution);
        shadows = true;
    } else if (quality == 0 && shadows) {
        shadowMap.reset();
        wideShadowMap.reset();
        shadows = false;
    }
    if (shadows && shadowMap->getResolution() != resolution) {
        shadowMap = std::make_unique<ShadowMap>(resolution);
        wideShadowMap = std::make_unique<ShadowMap>(resolution);
    }
    this->quality = quality;
}

void Shadows::setup(Shader& shader, const Weather& weather) {
    if (shadows) {
        const auto& worldInfo = level.getWorld()->getInfo();
        float cloudsIntensity = glm::max(worldInfo.fog, weather.clouds());
        shader.uniform1i("u_screen", 0);
        shader.uniformMatrix("u_shadowsMatrix[0]", shadowCamera.getProjView());
        shader.uniformMatrix("u_shadowsMatrix[1]", wideShadowCamera.getProjView());
        shader.uniform3f("u_sunDir", shadowCamera.front);
        shader.uniform1i("u_shadowsRes", shadowMap->getResolution());
        shader.uniform1f("u_shadowsOpacity", 1.0f - cloudsIntensity); // TODO: make it configurable
        shader.uniform1f("u_shadowsSoftness", 1.0f + cloudsIntensity * 4); // TODO: make it configurable

        glActiveTexture(GL_TEXTURE0 + TARGET_SHADOWS0);
        shader.uniform1i("u_shadows[0]", TARGET_SHADOWS0);
        glBindTexture(GL_TEXTURE_2D, shadowMap->getDepthMap());

        glActiveTexture(GL_TEXTURE0 + TARGET_SHADOWS1);
        shader.uniform1i("u_shadows[1]", TARGET_SHADOWS1);
        glBindTexture(GL_TEXTURE_2D, wideShadowMap->getDepthMap());

        glActiveTexture(TEXTURE_MAIN);
    }
}

void Shadows::refresh(const Camera& camera, const DrawContext& pctx, std::function<void(Camera&)> renderShadowPass) {
    static int frameid = 0;
    if (shadows) {
        if (frameid % 2 == 0) {
            generateShadowsMap(camera, pctx, *shadowMap, shadowCamera, 1.0f, renderShadowPass);
        } else {
            generateShadowsMap(camera, pctx, *wideShadowMap, wideShadowCamera, 3.0f, renderShadowPass);
        }
    }
    frameid++;
}

void Shadows::generateShadowsMap(
    const Camera& camera,
    const DrawContext& pctx,
    ShadowMap& shadowMap,
    Camera& shadowCamera,
    float scale,
    std::function<void(Camera&)> renderShadowPass
) {
    auto world = level.getWorld();
    const auto& worldInfo = world->getInfo();

    int resolution = shadowMap.getResolution();
    float shadowMapScale = 0.32f / (1 << glm::max(0, quality)) * scale;
    float shadowMapSize = resolution * shadowMapScale;

    glm::vec3 basePos = glm::floor(camera.position / 4.0f) * 4.0f;
    glm::vec3 prevPos = shadowCamera.position;
    shadowCamera = Camera(
        glm::distance2(prevPos, basePos) > 25.0f ? basePos : prevPos,
        shadowMapSize
    );
    shadowCamera.near = 0.1f;
    shadowCamera.far = 1000.0f;
    shadowCamera.perspective = false;
    shadowCamera.setAspectRatio(1.0f);

    float t = worldInfo.daytime - 0.25f;
    if (t < 0.0f) {
        t += 1.0f;
    }
    t = fmod(t, 0.5f);

    float sunCycleStep = 1.0f / 500.0f;
    float sunAngle = glm::radians(
        90.0f -
        ((static_cast<int>(t / sunCycleStep)) * sunCycleStep + 0.25f) * 360.0f
    );
    float sunAltitude = glm::pi<float>() * 0.25f;
    shadowCamera.rotate(
        -glm::cos(sunAngle + glm::pi<float>() * 0.5f) * sunAltitude,
        sunAngle - glm::pi<float>() * 0.5f,
        glm::radians(0.0f)
    );

    shadowCamera.position -= shadowCamera.front * 500.0f;
    shadowCamera.position += shadowCamera.up * 0.0f;
    shadowCamera.position += camera.front * 0.0f;

    auto view = shadowCamera.getView();

    auto currentPos = shadowCamera.position;
    auto topRight = shadowCamera.right + shadowCamera.up;
    auto min = view * glm::vec4(currentPos - topRight * shadowMapSize * 0.5f, 1.0f);
    auto max = view * glm::vec4(currentPos + topRight * shadowMapSize * 0.5f, 1.0f);

    shadowCamera.setProjection(glm::ortho(min.x, max.x, min.y, max.y, 0.1f, 1000.0f));

    {
        auto sctx = pctx.sub();
        sctx.setDepthTest(true);
        sctx.setCullFace(true);
        sctx.setViewport({resolution, resolution});
        shadowMap.bind();
        if (renderShadowPass) {
            renderShadowPass(shadowCamera);
        }
        shadowMap.unbind();
    }
}
