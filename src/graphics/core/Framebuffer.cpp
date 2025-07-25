#include "Framebuffer.hpp"

#include <GL/glew.h>
#include "Texture.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("gl-framebuffer");

Framebuffer::Framebuffer(uint fbo, uint depth, std::unique_ptr<Texture> texture)
  : fbo(fbo), depth(depth), texture(std::move(texture)) 
{
    if (this->texture) {
        width = this->texture->getWidth();
        height = this->texture->getHeight();
    } else {
        width = 0;
        height = 0;
    }
}

static std::unique_ptr<Texture> create_texture(int width, int height, int format) {
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    return std::make_unique<Texture>(tex, width, height);
}

Framebuffer::Framebuffer(uint width, uint height, bool alpha) 
  : width(width), height(height)
{
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    format = alpha ? GL_RGBA : GL_RGB;

    // Setup color attachment (texture)
    texture = create_texture(width, height, format);

    // Setup depth attachment
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "framebuffer is not complete!";
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Framebuffer::~Framebuffer() {
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &depth);
}

void Framebuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

void Framebuffer::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::resize(uint width, uint height) {
    if (this->width == width && this->height == height) {
        return;
    }
    this->width = width;
    this->height = height;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    texture = create_texture(width, height, format);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Texture* Framebuffer::getTexture() const {
    return texture.get();
}

uint Framebuffer::getWidth() const {
    return width;
}

uint Framebuffer::getHeight() const {
    return height;
}

uint Framebuffer::getFBO() const {
    return fbo;   
}
