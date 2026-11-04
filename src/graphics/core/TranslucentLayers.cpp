#include "TranslucentLayers.hpp"

#include <GL/glew.h>
#include <string>

#include "Mesh.hpp"
#include "PostProcessing.hpp"
#include "Shader.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("gl-translucent-layers");

inline constexpr int FIRST_TEXTURE_UNIT = advanced_pipeline::TARGETS_COUNT;

static void setup_texture(uint texture) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

TranslucentLayers::Layer::Layer() {
    glGenFramebuffers(1, &fbo);
    glGenTextures(1, &color);
    glGenTextures(1, &depth);
}

TranslucentLayers::Layer::~Layer() {
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &color);
    glDeleteTextures(1, &depth);
}

void TranslucentLayers::Layer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

void TranslucentLayers::Layer::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void TranslucentLayers::Layer::allocate(uint width, uint height) {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    setup_texture(color);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, nullptr
    );
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0
    );

    setup_texture(depth);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
        GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr
    );
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0
    );

    GLenum attachments[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, attachments);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "translucent layer framebuffer is not complete!";
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

TranslucentLayers::TranslucentLayers(size_t count, uint width, uint height)
    : width(width), height(height) {
    for (size_t i = 0; i < count; i++) {
        auto layer = std::make_unique<Layer>();
        layer->allocate(width, height);
        layers.push_back(std::move(layer));
    }
    PostProcessingVertex meshData[] {
        {{-1.0f, -1.0f}},
        {{-1.0f, 1.0f}},
        {{1.0f, 1.0f}},
        {{-1.0f, -1.0f}},
        {{1.0f, 1.0f}},
        {{1.0f, -1.0f}},
    };
    quadMesh = std::make_unique<Mesh<PostProcessingVertex>>(meshData, 6);
}

TranslucentLayers::~TranslucentLayers() = default;

void TranslucentLayers::resize(uint width, uint height) {
    if (this->width == width && this->height == height) {
        return;
    }
    this->width = width;
    this->height = height;
    for (auto& layer : layers) {
        layer->allocate(width, height);
    }
}

void TranslucentLayers::prepare(uint sourceFbo) {
    const GLfloat transparent[4] {0.0f, 0.0f, 0.0f, 0.0f};
    for (const auto& layer : layers) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, sourceFbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, layer->fbo);
        glBlitFramebuffer(
            0, 0, width, height, 0, 0, width, height,
            GL_DEPTH_BUFFER_BIT, GL_NEAREST
        );
        glBindFramebuffer(GL_FRAMEBUFFER, layer->fbo);
        glClearBufferfv(GL_COLOR, 0, transparent);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, sourceFbo);
}

Bindable& TranslucentLayers::getLayer(size_t index) {
    return *layers.at(index);
}

void TranslucentLayers::compose(Shader& shader) {
    shader.use();
    int count = static_cast<int>(layers.size());
    for (int i = 0; i < count; i++) {
        int colorUnit = FIRST_TEXTURE_UNIT + i;
        int depthUnit = FIRST_TEXTURE_UNIT + count + i;

        glActiveTexture(GL_TEXTURE0 + colorUnit);
        glBindTexture(GL_TEXTURE_2D, layers[i]->color);
        shader.uniform1i("u_layerColor" + std::to_string(i), colorUnit);

        glActiveTexture(GL_TEXTURE0 + depthUnit);
        glBindTexture(GL_TEXTURE_2D, layers[i]->depth);
        shader.uniform1i("u_layerDepth" + std::to_string(i), depthUnit);
    }
    quadMesh->draw();

    for (int i = 0; i < count * 2; i++) {
        glActiveTexture(GL_TEXTURE0 + FIRST_TEXTURE_UNIT + i);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glActiveTexture(GL_TEXTURE0);
}
