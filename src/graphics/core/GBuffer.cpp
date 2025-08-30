#include "GBuffer.hpp"

#include <GL/glew.h>

#include "debug/Logger.hpp"

using namespace advanced_pipeline;

static debug::Logger logger("gl-gbuffer");

// TODO: REFACTOR

void GBuffer::createColorBuffer() {
    if (colorBuffer == 0)
        glGenTextures(1, &colorBuffer);
    glBindTexture(GL_TEXTURE_2D, colorBuffer);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        width,
        height,
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorBuffer, 0
    );
}

void GBuffer::createPositionsBuffer() {
    if (positionsBuffer == 0)
        glGenTextures(1, &positionsBuffer);
    glBindTexture(GL_TEXTURE_2D, positionsBuffer);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA16F,
        width,
        height,
        0,
        GL_RGBA,
        GL_FLOAT,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, positionsBuffer, 0
    );
}

void GBuffer::createNormalsBuffer() {
    if (normalsBuffer == 0)
        glGenTextures(1, &normalsBuffer);
    glBindTexture(GL_TEXTURE_2D, normalsBuffer);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA16F,
        width,
        height,
        0,
        GL_RGB,
        GL_FLOAT,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, normalsBuffer, 0
    );
}

void GBuffer::createEmissionBuffer() {
    if (emissionBuffer == 0)
        glGenTextures(1, &emissionBuffer);
    glBindTexture(GL_TEXTURE_2D, emissionBuffer);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_R8,
        width,
        height,
        0,
        GL_RED,
        GL_FLOAT,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, emissionBuffer, 0
    );
}

void GBuffer::createDepthBuffer() {
    if (depthBuffer == 0)
        glGenRenderbuffers(1, &depthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer
    );
}

void GBuffer::createSSAOBuffer() {
    if (ssaoBuffer == 0)
        glGenTextures(1, &ssaoBuffer);
    glBindTexture(GL_TEXTURE_2D, ssaoBuffer);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_R16F,
        width,
        height,
        0,
        GL_RED,
        GL_FLOAT,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

GBuffer::GBuffer(uint width, uint height) : width(width), height(height) {
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    createColorBuffer();
    createPositionsBuffer();
    createNormalsBuffer();
    createEmissionBuffer();

    GLenum attachments[4] = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3,
    };
    glDrawBuffers(4, attachments);

    createDepthBuffer();

    int status;
    if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "gbuffer is not complete! (" << status << ")";
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenFramebuffers(1, &ssaoFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
    createSSAOBuffer();
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ssaoBuffer, 0
    );
    GLenum ssaoAttachments[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, ssaoAttachments);


    if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "SSAO framebuffer is not complete! (" << status << ")";
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GBuffer::~GBuffer() {
    glDeleteTextures(1, &colorBuffer);
    glDeleteTextures(1, &positionsBuffer);
    glDeleteTextures(1, &normalsBuffer);
    glDeleteTextures(1, &emissionBuffer);
    glDeleteTextures(1, &ssaoBuffer);
    glDeleteRenderbuffers(1, &depthBuffer);
    glDeleteFramebuffers(1, &fbo);
    glDeleteFramebuffers(1, &ssaoFbo);
}

void GBuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClear(GL_COLOR_BUFFER_BIT);
}

void GBuffer::bindSSAO() const {
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
}

void GBuffer::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::bindBuffers() const {
    glActiveTexture(GL_TEXTURE0 + TARGET_EMISSION);
    glBindTexture(GL_TEXTURE_2D, emissionBuffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_NORMALS);
    glBindTexture(GL_TEXTURE_2D, normalsBuffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_POSITIONS);
    glBindTexture(GL_TEXTURE_2D, positionsBuffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_COLOR);
    glBindTexture(GL_TEXTURE_2D, colorBuffer);

    if (TARGET_COLOR != 0)
        glActiveTexture(GL_TEXTURE0);
}

void GBuffer::bindSSAOBuffer() const {
    glBindTexture(GL_TEXTURE_2D, ssaoBuffer);
}

void GBuffer::bindDepthBuffer(int drawFbo) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFbo);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, 
                    GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

void GBuffer::resize(uint width, uint height) {
    if (this->width == width && this->height == height) {
        return;
    }
    this->width = width;
    this->height = height;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    createDepthBuffer();
    createColorBuffer();
    createPositionsBuffer();
    createNormalsBuffer();
    createEmissionBuffer();

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
    createSSAOBuffer();
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

std::unique_ptr<ImageData> GBuffer::toImage() const {
    auto data = std::make_unique<ubyte[]>(width * height * 3);
    glBindTexture(GL_TEXTURE_2D, colorBuffer);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data.get());
    glBindTexture(GL_TEXTURE_2D, 0);
    return std::make_unique<ImageData>(
        ImageFormat::rgb888, width, height, std::move(data)
    );
}

uint GBuffer::getWidth() const {
    return width;
}

uint GBuffer::getHeight() const {
    return height;
}
