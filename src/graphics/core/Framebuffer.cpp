#include "Framebuffer.hpp"

#include <GL/glew.h>

#include "Texture.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("gl-framebuffer");

Framebuffer::Framebuffer(uint fbo, uint depth, std::unique_ptr<Texture> texture)
    : fbo(fbo), depth(depth), texture(std::move(texture)) {
    if (this->texture) {
        width = this->texture->getWidth();
        height = this->texture->getHeight();
    } else {
        width = 0;
        height = 0;
    }
}

static std::unique_ptr<Texture> create_texture(
    int width, int height, bool alpha
) {
    GLenum glformat = alpha ? GL_RGBA : GL_RGB;

    GLuint textureid;
    glGenTextures(1, &textureid);
    glBindTexture(GL_TEXTURE_2D, textureid);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        glformat,
        width,
        height,
        0,
        glformat,
        GL_UNSIGNED_BYTE,
        nullptr
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureid, 0
    );
    return std::make_unique<Texture>(
        textureid,
        width,
        height,
        alpha ? ImageFormat::RGBA8888 : ImageFormat::RGB888
    );
}

Framebuffer::Framebuffer(uint width, uint height, bool alpha)
    : width(width), height(height) {
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Setup color attachment (texture)
    texture = create_texture(width, height, alpha);

    // Setup depth attachment
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth
    );

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        logger.error() << "framebuffer is not complete!";
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Framebuffer::~Framebuffer() {
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &depth);
}

void Framebuffer::setTexture(std::unique_ptr<Texture> texture) {
    this->texture = std::move(texture);
}

void Framebuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

void Framebuffer::unbind() const {
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

    texture->resize(width, height);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

Texture* Framebuffer::getTexture() const {
    return texture.get();
}

std::shared_ptr<Texture> Framebuffer::getSharedTexture() const {
    return texture;
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
