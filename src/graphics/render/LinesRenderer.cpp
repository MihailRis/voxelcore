#include "LinesRenderer.hpp"

#include "graphics/core/LineBatch.hpp"

void LinesRenderer::draw(LineBatch& batch) {
    for (const auto& line : queue) {
        batch.line(line.a, line.b, line.color);
    }
    queue.clear();
}

void LinesRenderer::pushLine(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color) {
    queue.push_back({a, b, color});
}
