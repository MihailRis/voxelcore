#include "Texture.hpp"
#include "gl_util.hpp"

#include <GL/glew.h>
#include <stdexcept>
#include <memory>

uint Texture::MAX_RESOLUTION = 1024; // Window.initialize overrides it

Texture::Texture(uint id, uint width, uint height) 
    : id(id), width(width), height(height) {
}

Texture::Texture(const ubyte* data, uint width, uint height, ImageFormat imageFormat) 
    : width(width), height(height) {
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    GLenum format = gl::to_glenum(imageFormat);
    glTexImage2D(
        GL_TEXTURE_2D, 0, format, width, height, 0,
        format, GL_UNSIGNED_BYTE, static_cast<const GLvoid*>(data)
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 1);
    glBindTexture(GL_TEXTURE_2D, 0);
}

Texture::~Texture() {
    glDeleteTextures(1, &id);
}

void Texture::bind() const {
    glBindTexture(GL_TEXTURE_2D, id);
}

void Texture::unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::reload(const ImageData& image) {
    width = image.getWidth();
    height = image.getHeight();
    reload(image.getData(), width, height);
}

void Texture::reload(const ubyte* data, uint width, uint height) {
    glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, static_cast<const GLvoid*>(data));
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::reloadPartial(const ImageData& image, uint x, uint y, uint w, uint h) {
    glBindTexture(GL_TEXTURE_2D, id);
    glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, image.getData());
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

std::unique_ptr<ImageData> Texture::readData() {
    auto data = std::make_unique<ubyte[]>(width * height * 4);
    glBindTexture(GL_TEXTURE_2D, id);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.get());
    glBindTexture(GL_TEXTURE_2D, 0);
    return std::make_unique<ImageData>(
        ImageFormat::rgba8888, width, height, std::move(data)
    );
}

void Texture::setNearestFilter() {
    bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::setMipMapping(bool flag, bool pixelated) {
    bind();
    if (flag) {
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_MIN_FILTER,
            pixelated ? GL_NEAREST : GL_LINEAR_MIPMAP_NEAREST
        );
    } else {
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_MIN_FILTER,
            pixelated ? GL_NEAREST : GL_LINEAR
        );
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}

std::unique_ptr<Texture> Texture::from(const ImageData* image) {
    uint width = image->getWidth();
    uint height = image->getHeight();
    void* data = image->getData();
    return std::make_unique<Texture>(
        static_cast<ubyte*>(data), width, height, image->getFormat()
    );
}

uint Texture::getId() const {
    return id;
}
