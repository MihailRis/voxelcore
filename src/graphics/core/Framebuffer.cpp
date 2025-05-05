#include "Framebuffer.hpp"

#include <GL/glew.h>
#include "Texture.hpp"

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
#include <iostream>
Framebuffer::Framebuffer(uint width, uint height, bool alpha) 
  : width(width), height(height)
{
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    format = alpha ? GL_RGBA : GL_RGB;

    // Setup color attachment (texture)
    texture = create_texture(width, height, format);

    glGenTextures(1, &positions);
    glBindTexture(GL_TEXTURE_2D, positions);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, positions, 0);

    glGenTextures(1, &normals);
    glBindTexture(GL_TEXTURE_2D, normals);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, normals, 0);

    unsigned int attachments[3] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
    glDrawBuffers(3, attachments);

    // Setup depth attachment
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "Framebuffer not complete!" << std::endl;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Framebuffer::~Framebuffer() {
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &normals);
    glDeleteTextures(1, &depth);
}

void Framebuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

void Framebuffer::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::bindBuffers() {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->getId());

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, positions);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, normals);

    glActiveTexture(GL_TEXTURE0);
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

    glBindTexture(GL_TEXTURE_2D, positions);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, positions, 0);

    glBindTexture(GL_TEXTURE_2D, normals);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, normals, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

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
