#include "DrawContext.hpp"

#include <GL/glew.h>

#include <utility>

#include "Batch2D.hpp"
#include "Framebuffer.hpp"
#include "window/Window.hpp"

static void set_blend_mode(BlendMode mode) {
    switch (mode) {
        case BlendMode::normal:
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            break;
        case BlendMode::addition:
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            break;
        case BlendMode::inversion:
            glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ONE_MINUS_SRC_ALPHA);
            break;
    }
}


DrawContext::DrawContext(
    const DrawContext* parent, 
    Window& window,
    Batch2D* g2d
) : window(window),
    parent(parent), 
    viewport(window.getSize()),
    g2d(g2d),
    flushable(g2d)
{}

DrawContext::~DrawContext() {
    if (flushable) {
        flushable->flush();
    }

    while (scissorsCount--) {
        window.popScissor();
    }

    if (parent == nullptr)
        return;

    if (fbo != parent->fbo) {
        if (fbo) {
            fbo->unbind();
        }
        if (parent->fbo) {
            parent->fbo->bind();
        }
    }

    glViewport(0, 0, parent->viewport.x, parent->viewport.y);

    if (depthMask != parent->depthMask) {
        glDepthMask(parent->depthMask);
    }
    if (depthTest != parent->depthTest) {
        if (depthTest) glDisable(GL_DEPTH_TEST);
        else glEnable(GL_DEPTH_TEST);
    }
    if (cullFace != parent->cullFace) {
        if (cullFace) glDisable(GL_CULL_FACE);
        else glEnable(GL_CULL_FACE);
    }
    if (blendMode != parent->blendMode) {
        set_blend_mode(parent->blendMode);
    }
    if (lineWidth != parent->lineWidth) {
        glLineWidth(parent->lineWidth);
    }
}

const glm::uvec2& DrawContext::getViewport() const {
    return viewport;
}

Batch2D* DrawContext::getBatch2D() const {
    return g2d;
}

DrawContext DrawContext::sub(Flushable* flushable) const {
    auto ctx = DrawContext(*this);
    ctx.parent = this;
    ctx.flushable = flushable;
    ctx.scissorsCount = 0;
    if (auto batch2D = dynamic_cast<Batch2D*>(flushable)) {
        ctx.g2d = batch2D;
    }
    return ctx;
}

void DrawContext::setViewport(const glm::uvec2& viewport) {
    this->viewport = viewport;
    glViewport(0, 0, viewport.x, viewport.y);
}

void DrawContext::setFramebuffer(Bindable* fbo) {
    if (this->fbo == fbo)
        return;
    this->fbo = fbo;
    if (fbo) {
        fbo->bind();
    }
}

void DrawContext::setDepthMask(bool flag) {
    if (depthMask == flag)
        return;
    depthMask = flag;
    glDepthMask(GL_FALSE + flag);
}

void DrawContext::setDepthTest(bool flag) {
    if (depthTest == flag)
        return;
    depthTest = flag;
    if (flag) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
}

void DrawContext::setCullFace(bool flag) {
    if (cullFace == flag)
        return;
    cullFace = flag;
    if (flag) {
        glEnable(GL_CULL_FACE);
    } else {
        glDisable(GL_CULL_FACE);
    }
}

void DrawContext::setBlendMode(BlendMode mode) {
    if (blendMode == mode)
        return;
    blendMode = mode;
    set_blend_mode(mode);
}

void DrawContext::setScissors(const glm::vec4& area) {
    window.pushScissor(area);
    scissorsCount++;
}

void DrawContext::setLineWidth(float width) {
    lineWidth = width;
    glLineWidth(width);
}
