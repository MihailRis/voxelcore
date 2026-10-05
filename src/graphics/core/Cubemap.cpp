#include "Cubemap.hpp"
#include "gl_util.hpp"

#include <GL/glew.h>

Cubemap::Cubemap(uint width, uint height, ImageFormat imageFormat) 
  : Texture(0U, width, height, imageFormat) 
{
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_CUBE_MAP, id);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    for (uint face = 0; face < 6; face++) {
        glTexImage2D(
            GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 
            0, 
            format, 
            width, 
            height, 
            0, 
            format, 
            GL_UNSIGNED_BYTE, 
            nullptr
        );
    }
}

void Cubemap::bind() const {
    glBindTexture(GL_TEXTURE_CUBE_MAP, id);
}

void Cubemap::unbind() const {
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}

void Cubemap::reload(const ubyte* data, uint width, uint height) {
    this->width = width;
    this->height = height;
    glBindTexture(GL_TEXTURE_CUBE_MAP, id);
    for (uint face = 0; face < 6; face++) {
        glTexImage2D(
            GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 
            0, 
            format, 
            width, 
            height, 
            0, 
            format, 
            GL_UNSIGNED_BYTE, 
            data
        );
    }
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}
