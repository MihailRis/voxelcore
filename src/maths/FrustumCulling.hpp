#pragma once

#include <glm/matrix.hpp>

class Frustum {
public:
    Frustum() = default;

    void update(glm::mat4 projview);
    bool isBoxVisible(const glm::vec3& minp, const glm::vec3& maxp) const;
private:
    enum Planes {
        Left = 0,
        Right,
        Bottom,
        Top,
        Near,
        Far,
        Count,
        Combinations = Count * (Count - 1) / 2
    };

    template <Planes i, Planes j>
    struct ij2k {
        enum { k = i * (9 - i) / 2 + j - 1 };
    };

    template <Planes a, Planes b, Planes c>
    glm::vec3 intersection(const glm::vec3* crosses) const;

    glm::vec4 planes[Count];
    glm::vec3 points[8];
    glm::vec3 minPoint;
    glm::vec3 maxPoint;
};

inline void Frustum::update(glm::mat4 m) {
    m = glm::transpose(m);
    planes[Left] = m[3] + m[0];
    planes[Right] = m[3] - m[0];
    planes[Bottom] = m[3] + m[1];
    planes[Top] = m[3] - m[1];
    planes[Near] = m[3] + m[2];
    planes[Far] = m[3] - m[2];

    glm::vec3 crosses[Combinations] = {
        glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Right])),
        glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Bottom])),
        glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Top])),
        glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Near])),
        glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Far])),
        glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Bottom])),
        glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Top])),
        glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Near])),
        glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Far])),
        glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Top])),
        glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Near])),
        glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Far])),
        glm::cross(glm::vec3(planes[Top]), glm::vec3(planes[Near])),
        glm::cross(glm::vec3(planes[Top]), glm::vec3(planes[Far])),
        glm::cross(glm::vec3(planes[Near]), glm::vec3(planes[Far]))};

    points[0] = intersection<Left, Bottom, Near>(crosses);
    points[1] = intersection<Left, Top, Near>(crosses);
    points[2] = intersection<Right, Bottom, Near>(crosses);
    points[3] = intersection<Right, Top, Near>(crosses);
    points[4] = intersection<Left, Bottom, Far>(crosses);
    points[5] = intersection<Left, Top, Far>(crosses);
    points[6] = intersection<Right, Bottom, Far>(crosses);
    points[7] = intersection<Right, Top, Far>(crosses);

    minPoint = maxPoint = points[0];
    for (int i = 1; i < 8; i++) {
        minPoint = glm::min(minPoint, points[i]);
        maxPoint = glm::max(maxPoint, points[i]);
    }
}

inline bool Frustum::isBoxVisible(const glm::vec3& minp, const glm::vec3& maxp)
    const {
    for (int i = 0; i < Count; i++) {
        const glm::vec4& p = planes[i];
        glm::vec3 v(
            p.x > 0.0f ? maxp.x : minp.x,
            p.y > 0.0f ? maxp.y : minp.y,
            p.z > 0.0f ? maxp.z : minp.z
        );
        if (glm::dot(glm::vec3(p), v) + p.w < 0.0f) {
            return false;
        }
    }

    return !(minPoint.x > maxp.x || maxPoint.x < minp.x ||
             minPoint.y > maxp.y || maxPoint.y < minp.y ||
             minPoint.z > maxp.z || maxPoint.z < minp.z);
}

template <Frustum::Planes a, Frustum::Planes b, Frustum::Planes c>
inline glm::vec3 Frustum::intersection(const glm::vec3* crosses) const {
    float d = glm::dot(glm::vec3(planes[a]), crosses[ij2k<b, c>::k]);
    glm::vec3 res = glm::mat3(
                        crosses[ij2k<b, c>::k],
                        -crosses[ij2k<a, c>::k],
                        crosses[ij2k<a, b>::k]
                    ) *
                    glm::vec3(planes[a].w, planes[b].w, planes[c].w);
    return res * (-1.0f / d);
}
