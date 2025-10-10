#include "TextureAnimation.hpp"
#include "Texture.hpp"
#include "Framebuffer.hpp"

#include <GL/glew.h>
#include <unordered_set>

TextureAnimator::TextureAnimator() {
    glGenFramebuffers(1, &fboR);
    glGenFramebuffers(1, &fboD);
}

TextureAnimator::~TextureAnimator() {
    glDeleteFramebuffers(1, &fboR);
    glDeleteFramebuffers(1, &fboD);
}

void TextureAnimator::addAnimations(const std::vector<TextureAnimation>& animations) {
    for (const auto& elem : animations) {
        addAnimation(elem);
    }
}

void TextureAnimator::update(float delta) {
    std::unordered_set<uint> changedTextures;

    for (auto& elem : animations) {
        if (elem.frames.empty()) {
            continue;
        }
        elem.timer += delta;
        size_t frameNum = elem.currentFrame;
        Frame frame = elem.frames[elem.currentFrame];
        while (elem.timer >= frame.duration) {
            elem.timer -= frame.duration;
            elem.currentFrame++;
            if (elem.currentFrame >= elem.frames.size()) elem.currentFrame = 0;
            frame = elem.frames[elem.currentFrame];
        }
        if (frameNum != elem.currentFrame){
            uint elemDstId = elem.dstTexture->getId();
            uint elemSrcId = elem.srcTexture->getId();

            changedTextures.insert(elemDstId);

            glBindFramebuffer(GL_FRAMEBUFFER, fboD);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, elemDstId, 0);

            glBindFramebuffer(GL_FRAMEBUFFER, fboR);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, elemSrcId, 0);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboD);
            glBindFramebuffer(GL_READ_FRAMEBUFFER, fboR);

            float srcPosY = elem.srcTexture->getHeight() - frame.size.y - frame.srcPos.y; // vertical flip

            glBlitFramebuffer(
                frame.srcPos.x, srcPosY,
                frame.srcPos.x + frame.size.x,    
                srcPosY + frame.size.y,
                frame.dstPos.x, frame.dstPos.y,    
                frame.dstPos.x + frame.size.x,
                frame.dstPos.y + frame.size.y,
                GL_COLOR_BUFFER_BIT, GL_NEAREST
            );
        }
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    for (auto& elem : changedTextures) {
        glBindTexture(GL_TEXTURE_2D, elem);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}
