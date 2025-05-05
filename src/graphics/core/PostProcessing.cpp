#include "PostProcessing.hpp"
#include "Mesh.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "Framebuffer.hpp"
#include "DrawContext.hpp"
#include "PostEffect.hpp"
#include "assets/Assets.hpp"
#include "window/Camera.hpp"

#include <stdexcept>
#include <random>

#include <iostream>

PostProcessing::PostProcessing(size_t effectSlotsCount)
    : effectSlots(effectSlotsCount) {
    // Fullscreen quad mesh bulding
    PostProcessingVertex meshData[]{
            {{-1.0f, -1.0f}},
            {{-1.0f, 1.0f}},
            {{1.0f, 1.0f}},
            {{-1.0f, -1.0f}},
            {{1.0f, 1.0f}},
            {{1.0f, -1.0f}},
    };

    quadMesh = std::make_unique<Mesh<PostProcessingVertex>>(meshData, 6);

    std::vector<glm::vec3> ssaoNoise;
    for (unsigned int i = 0; i < 16; i++)
    {
        glm::vec3 noise(
            (rand() / static_cast<float>(RAND_MAX)) * 2.0 - 1.0, 
            (rand() / static_cast<float>(RAND_MAX)) * 2.0 - 1.0, 
            0.0f); 
        ssaoNoise.push_back(noise);
    }  
    glGenTextures(1, &noiseTexture);
    glBindTexture(GL_TEXTURE_2D, noiseTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, ssaoNoise.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);

    std::uniform_real_distribution<float> randomFloats(0.0, 1.0); 
    std::default_random_engine generator;
    for (unsigned int i = 0; i < 64; ++i)
    {
        glm::vec3 sample(
            randomFloats(generator) * 2.0 - 1.0, 
            randomFloats(generator) * 2.0 - 1.0, 
            randomFloats(generator)
        );
        sample  = glm::normalize(sample);
        sample *= randomFloats(generator);
        float scale = (float)i / 64.0; 
        ssaoKernel.push_back(sample);  
    }
}

PostProcessing::~PostProcessing() = default;

void PostProcessing::use(DrawContext& context) {
    const auto& vp = context.getViewport();
    if (fbo) {
        fbo->resize(vp.x, vp.y);
        fboSecond->resize(vp.x, vp.y);
    } else {
        fbo = std::make_unique<Framebuffer>(vp.x, vp.y);
        fboSecond = std::make_unique<Framebuffer>(vp.x, vp.y);
    }
    context.setFramebuffer(fbo.get());
}

void PostProcessing::render(
    const DrawContext& context, const Assets& assets, float timer, const Camera& camera, uint depthMap
) {
    if (fbo == nullptr) {
        throw std::runtime_error("'use(...)' was never called");
    }
    int totalPasses = 0;
    for (const auto& effect : effectSlots) {
        totalPasses += (effect != nullptr && effect->isActive());
    }

    if (totalPasses == 0) {
        auto& effect = assets.require<PostEffect>("default");
        fbo->getTexture()->bind();
        fbo->bindBuffers();

        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, noiseTexture);
        glActiveTexture(GL_TEXTURE0);

        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glActiveTexture(GL_TEXTURE0);

        const auto& viewport = context.getViewport();

        auto& shader = effect.use();
        if (!ssaoConfigured) {
            for (unsigned int i = 0; i < 64; ++i) {
                auto name = "samples["+ std::to_string(i) + "]";
                shader.uniform3f(name, ssaoKernel[i]);
            }
            ssaoConfigured = true;
        }
        shader.uniform1i("u_screen", 0);
        shader.uniform1i("u_position", 1);
        shader.uniform1i("u_normal", 2);
        shader.uniform1i("u_noise", 3);
        shader.uniform1i("u_shadows", 4);
        shader.uniform2i("u_screenSize", viewport);
        shader.uniform1f("u_timer", timer);
        shader.uniform1f("near_plane", camera.near);
        shader.uniform1f("far_plane", camera.far);
        shader.uniformMatrix("u_projection", camera.getProjection());

        quadMesh->draw();
        return;
    }

    int currentPass = 1;
    for (const auto& effect : effectSlots) {
        if (effect == nullptr || !effect->isActive()) {
            continue;
        }
        auto& shader = effect->use();

        for (unsigned int i = 0; i < 64; ++i) {
            shader.uniform3f("samples["+ std::to_string(i) + "]", ssaoKernel[i]);
        }

        const auto& viewport = context.getViewport();
        shader.uniform1i("u_screen", 0);
        shader.uniform1i("u_position", 1);
        shader.uniform1i("u_normal", 2);
        shader.uniform1i("u_noise", 3);
        shader.uniform2i("u_screenSize", viewport);
        shader.uniform1f("u_timer", timer);
        shader.uniformMatrix("u_projection", camera.getProjView(false));
        fbo->getTexture()->bind();
        fbo->bindBuffers();

        if (currentPass < totalPasses) {
            fboSecond->bind();
        }

        quadMesh->draw();
        if (currentPass < totalPasses) {
            fboSecond->unbind();
            std::swap(fbo, fboSecond);
        }
        currentPass++;
    }
}

void PostProcessing::setEffect(size_t slot, std::shared_ptr<PostEffect> effect) {
    effectSlots.at(slot) = std::move(effect);
}

PostEffect* PostProcessing::getEffect(size_t slot) {
    return effectSlots.at(slot).get();
}

std::unique_ptr<ImageData> PostProcessing::toImage() {
    return fbo->getTexture()->readData();
}

Framebuffer* PostProcessing::getFramebuffer() const {
    return fbo.get();
}
