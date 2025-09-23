#include "DebugLinesRenderer.hpp"

#include "graphics/core/Shader.hpp"
#include "window/Camera.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/render/LinesRenderer.hpp"
#include "world/Level.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Pathfinding.hpp"
#include "maths/voxmaths.hpp"

bool DebugLinesRenderer::showPaths = false;

static void draw_route(
    LinesRenderer& lines, const voxels::Agent& agent
) {
    const auto& route = agent.route;
    if (!route.found)
        return;

    for (int i = 1; i < route.nodes.size(); i++) {
        const auto& a = route.nodes.at(i - 1);
        const auto& b = route.nodes.at(i);

        if (i == 1) {
            lines.pushLine(
                glm::vec3(a.pos) + glm::vec3(0.5f),
                glm::vec3(a.pos) + glm::vec3(0.5f, 1.0f, 0.5f),
                glm::vec4(1, 1, 1, 1)
            );
        }

        lines.pushLine(
            glm::vec3(a.pos) + glm::vec3(0.5f),
            glm::vec3(b.pos) + glm::vec3(0.5f),
            glm::vec4(1, 0, 1, 1)
        );

        lines.pushLine(
            glm::vec3(b.pos) + glm::vec3(0.5f),
            glm::vec3(b.pos) + glm::vec3(0.5f, 1.0f, 0.5f),
            glm::vec4(1, 1, 1, 1)
        );
    }
}

void DebugLinesRenderer::drawBorders(
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

void DebugLinesRenderer::drawCoordSystem(
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


void DebugLinesRenderer::render(
    DrawContext& pctx,
    const Camera& camera,
    LinesRenderer& renderer,
    LineBatch& linesBatch,
    Shader& linesShader,
    bool showChunkBorders
) {
    // In-world lines
    if (showPaths) {
        for (const auto& [_, agent] : level.pathfinding->getAgents()) {
            draw_route(renderer, agent);
        }
    }
    DrawContext ctx = pctx.sub(&linesBatch);
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
            linesBatch,
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
    drawCoordSystem(linesBatch, ctx, length);
}
