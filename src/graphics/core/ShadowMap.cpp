#include "ShadowMap.hpp"

#include <GL/glew.h>

ShadowMap::ShadowMap(int resolution) {
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        resolution, resolution, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

ShadowMap::~ShadowMap() {
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &depthMap);
}

void ShadowMap::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void ShadowMap::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

uint ShadowMap::getDepthMap() {
    return depthMap;
}
