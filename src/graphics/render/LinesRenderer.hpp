#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <vector>

class LineBatch;

class LinesRenderer {
public:
    struct Line {
        glm::vec3 a;
        glm::vec3 b;
        glm::vec4 color;
    };

    void draw(LineBatch& batch);

    void pushLine(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color);
private:
    std::vector<Line> queue;
};
