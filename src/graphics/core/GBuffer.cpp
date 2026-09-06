#include "GBuffer.hpp"

#include <GL/glew.h>

#include "debug/Logger.hpp"

using namespace advanced_pipeline;

static debug::Logger logger("gl-gbuffer");

void GBuffer::createBuffer(Buffer& buffer) {
    if (buffer.buffer == 0) {
        if (&buffer == &depthBuffer)
            glGenRenderbuffers(1, &depthBuffer.buffer);
        else
            glGenTextures(1, &buffer.buffer);
    }
    if (&buffer == &depthBuffer){
        glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer.buffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(
            GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer.buffer
        );
    } else {
        glBindTexture(GL_TEXTURE_2D, buffer.buffer);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            buffer.internalformat,
            width,
            height,
            0,
            buffer.format,
            buffer.type,
            nullptr
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        if (&buffer != &ssaoBuffer)
        {
            glFramebufferTexture2D(
                GL_FRAMEBUFFER, buffer.attachment, GL_TEXTURE_2D, buffer.buffer, 0
            );
        }
    }
}

GBuffer::GBuffer(uint width, uint height)
    : width(width), height(height), 

      colorBuffer       {0, GL_RGB,     GL_RGB,  GL_UNSIGNED_BYTE, GL_COLOR_ATTACHMENT0},
      positionsBuffer   {0, GL_RGBA16F, GL_RGBA, GL_FLOAT,         GL_COLOR_ATTACHMENT1},
      normalsBuffer     {0, GL_RGBA16F, GL_RGB,  GL_FLOAT,         GL_COLOR_ATTACHMENT2},
      emissionBuffer    {0, GL_R8,      GL_RED,  GL_FLOAT,         GL_COLOR_ATTACHMENT3},
      depthBuffer       {0, 0,          0,       0,                0},
      ssaoBuffer        {0, GL_R16F,    GL_RED,  GL_FLOAT,         0}
{
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    createBuffer(colorBuffer);
    createBuffer(positionsBuffer);
    createBuffer(normalsBuffer);
    createBuffer(emissionBuffer);

    GLenum attachments[4] = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3,
    };
    glDrawBuffers(4, attachments);

    createBuffer(depthBuffer);

    int status;
    if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "gbuffer is not complete! (" << status << ")";
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenFramebuffers(1, &ssaoFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
    createBuffer(ssaoBuffer);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ssaoBuffer.buffer, 0
    );
    GLenum ssaoAttachments[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, ssaoAttachments);


    if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "SSAO framebuffer is not complete! (" << status << ")";
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GBuffer::~GBuffer() {
    glDeleteTextures(1, &colorBuffer.buffer);
    glDeleteTextures(1, &positionsBuffer.buffer);
    glDeleteTextures(1, &normalsBuffer.buffer);
    glDeleteTextures(1, &emissionBuffer.buffer);
    glDeleteTextures(1, &ssaoBuffer.buffer);
    glDeleteRenderbuffers(1, &depthBuffer.buffer);
    glDeleteFramebuffers(1, &fbo);
    glDeleteFramebuffers(1, &ssaoFbo);
}

void GBuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClear(GL_COLOR_BUFFER_BIT);
}

void GBuffer::bindSSAO() const {
    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
}

void GBuffer::unbind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::bindBuffers() const {
    glActiveTexture(GL_TEXTURE0 + TARGET_EMISSION);
    glBindTexture(GL_TEXTURE_2D, emissionBuffer.buffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_NORMALS);
    glBindTexture(GL_TEXTURE_2D, normalsBuffer.buffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_POSITIONS);
    glBindTexture(GL_TEXTURE_2D, positionsBuffer.buffer);

    glActiveTexture(GL_TEXTURE0 + TARGET_COLOR);
    glBindTexture(GL_TEXTURE_2D, colorBuffer.buffer);

    if (TARGET_COLOR != 0)
        glActiveTexture(GL_TEXTURE0);
}

void GBuffer::bindSSAOBuffer() const {
    glBindTexture(GL_TEXTURE_2D, ssaoBuffer.buffer);
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

    createBuffer(depthBuffer);
    createBuffer(colorBuffer);
    createBuffer(positionsBuffer);
    createBuffer(normalsBuffer);
    createBuffer(emissionBuffer);

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, ssaoFbo);
    createBuffer(ssaoBuffer);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

std::unique_ptr<ImageData> GBuffer::toImage() const {
    auto data = std::make_unique<ubyte[]>(width * height * 3);
    glBindTexture(GL_TEXTURE_2D, colorBuffer.buffer);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data.get());
    glBindTexture(GL_TEXTURE_2D, 0);
    return std::make_unique<ImageData>(
        ImageFormat::RGB888, width, height, std::move(data)
    );
}

uint GBuffer::getWidth() const {
    return width;
}

uint GBuffer::getHeight() const {
    return height;
}
