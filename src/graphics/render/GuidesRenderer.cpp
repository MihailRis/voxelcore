#include "GuidesRenderer.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include "graphics/core/Shader.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/core/DrawContext.hpp"
#include "maths/voxmaths.hpp"
#include "window/Camera.hpp"
#include "constants.hpp"

void GuidesRenderer::drawBorders(
    LineBatch& batch, int sx, int sy, int sz, int ex, int ey, int ez
) {
    int ww = ex - sx;
    int dd = ez - sz;
    /*corner*/ {
        batch.line(sx, sy, sz, sx, ey, sz, 0.8f, 0, 0.8f, 1);
        batch.line(sx, sy, ez, sx, ey, ez, 0.8f, 0, 0.8f, 1);
        batch.line(ex, sy, sz, ex, ey, sz, 0.8f, 0, 0.8f, 1);
        batch.line(ex, sy, ez, ex, ey, ez, 0.8f, 0, 0.8f, 1);
    }
    for (int i = 2; i < ww; i += 2) {
        batch.line(sx + i, sy, sz, sx + i, ey, sz, 0, 0, 0.8f, 1);
        batch.line(sx + i, sy, ez, sx + i, ey, ez, 0, 0, 0.8f, 1);
    }
    for (int i = 2; i < dd; i += 2) {
        batch.line(sx, sy, sz + i, sx, ey, sz + i, 0.8f, 0, 0, 1);
        batch.line(ex, sy, sz + i, ex, ey, sz + i, 0.8f, 0, 0, 1);
    }
    for (int i = sy; i < ey; i += 2) {
        batch.line(sx, i, sz, sx, i, ez, 0, 0.8f, 0, 1);
        batch.line(sx, i, ez, ex, i, ez, 0, 0.8f, 0, 1);
        batch.line(ex, i, ez, ex, i, sz, 0, 0.8f, 0, 1);
        batch.line(ex, i, sz, sx, i, sz, 0, 0.8f, 0, 1);
    }
    batch.flush();
}

void GuidesRenderer::drawCoordSystem(
    LineBatch& batch, const DrawContext& pctx, float length
) {
    auto ctx = pctx.sub();
    ctx.setDepthTest(false);
    batch.lineWidth(4.0f);
    batch.line(0.f, 0.f, 0.f, length, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f);
    batch.line(0.f, 0.f, 0.f, 0.f, length, 0.f, 0.f, 0.f, 0.f, 1.f);
    batch.line(0.f, 0.f, 0.f, 0.f, 0.f, length, 0.f, 0.f, 0.f, 1.f);
    batch.flush();

    ctx.setDepthTest(true);
    batch.lineWidth(2.0f);
    batch.line(0.f, 0.f, 0.f, length, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f);
    batch.line(0.f, 0.f, 0.f, 0.f, length, 0.f, 0.f, 1.f, 0.f, 1.f);
    batch.line(0.f, 0.f, 0.f, 0.f, 0.f, length, 0.f, 0.f, 1.f, 1.f);
}

void GuidesRenderer::renderDebugLines(
    const DrawContext& pctx,
    const Camera& camera,
    LineBatch& batch,
    Shader& linesShader,
    bool showChunkBorders
) {
    DrawContext ctx = pctx.sub(&batch);
    const auto& viewport = ctx.getViewport();

    ctx.setDepthTest(true);

    linesShader.use();

    if (showChunkBorders) {
        linesShader.uniformMatrix("u_projview", camera.getProjView());
        glm::vec3 coord = camera.position;
        if (coord.x < 0) coord.x--;
        if (coord.z < 0) coord.z--;
        int cx = floordiv(static_cast<int>(coord.x), CHUNK_W);
        int cz = floordiv(static_cast<int>(coord.z), CHUNK_D);

        drawBorders(
            batch,
            cx * CHUNK_W,
            0,
            cz * CHUNK_D,
            (cx + 1) * CHUNK_W,
            CHUNK_H,
            (cz + 1) * CHUNK_D
        );
    }

    float length = 40.f;
    glm::vec3 tsl(viewport.x / 2, viewport.y / 2, 0.f);
    glm::mat4 model(glm::translate(glm::mat4(1.f), tsl));
    linesShader.uniformMatrix(
        "u_projview",
        glm::ortho(
            0.f,
            static_cast<float>(viewport.x),
            0.f,
            static_cast<float>(viewport.y),
            -length,
            length
        ) * model *
            glm::inverse(camera.rotation)
    );
    drawCoordSystem(batch, ctx, length);
}
