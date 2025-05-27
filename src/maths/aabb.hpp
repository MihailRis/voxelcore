#pragma once

#include <glm/glm.hpp>

/// @brief Axis Aligned Bounding Box
struct AABB {
    glm::vec3 a {0.0f};
    glm::vec3 b {1.0f, 1.0f, 1.0f};

    AABB() = default;

    AABB(glm::vec3 size) : a(0.0f), b(size) {
    }

    AABB(glm::vec3 a, glm::vec3 b) : a(a), b(b) {
    }

    /// @brief Get AABB point with minimal x,y,z
    inline glm::vec3 min() const {
        return glm::min(a, b);
    }

    /// @brief Get AABB point with minimal x,y,z
    inline glm::vec3 max() const {
        return glm::max(a, b);
    }

    /// @brief Get AABB dimensions: width, height, depth
    inline glm::vec3 size() const {
        return glm::vec3(fabs(b.x - a.x), fabs(b.y - a.y), fabs(b.z - a.z));
    }

    /// @brief Get AABB center
    inline glm::vec3 center() const {
        return (a + b) * 0.5f;
    }

    /// @brief Move AABB
    inline AABB translated(const glm::vec3& pos) const {
        return AABB(a + pos, b + pos);
    }

    /// @brief Increment AABB size from center
    inline void increment(const glm::vec3 add) {
        glm::vec3 center = (a + b) * 0.5f;
        a = (a - center) - add * 0.5f + center;
        b = (b - center) + add * 0.5f + center;
    }

    /// @brief Multiply AABB size from center
    inline void scale(const glm::vec3 mul) {
        glm::vec3 center = (a + b) * 0.5f;
        a = (a - center) * mul + center;
        b = (b - center) * mul + center;
    }

    /// @brief Multiply AABB size from given origin
    inline void scale(const glm::vec3 mul, const glm::vec3 orig) {
        glm::vec3 beg = min();
        glm::vec3 end = max();
        glm::vec3 center = glm::mix(beg, end, orig);
        a = (a - center) * mul + center;
        b = (b - center) * mul + center;
    }

    /// @brief Check if given point is inside
    inline bool contains(const glm::vec3 pos) const {
        const glm::vec3 p = min();
        const glm::vec3 s = size();
        return !(
            pos.x < p.x || pos.y < p.y || pos.z < p.z || pos.x >= p.x + s.x ||
            pos.y >= p.y + s.y || pos.z >= p.z + s.z
        );
    }

    void fix() {
        auto beg = min();
        auto end = max();
        a = beg;
        b = end;
    }

    inline void addPoint(glm::vec3 p) {
        a = glm::min(a, p);
        b = glm::max(b, p);
    }

    /// TODO: optimize
    void transform(const glm::mat4& matrix) {
        auto pa = a;
        auto pb = b;
        a = matrix * glm::vec4(a, 1.0f);
        b = matrix * glm::vec4(b, 1.0f);
        fix();
        addPoint(matrix * glm::vec4(pb.x, pa.y, pa.z, 1.0f));
        addPoint(matrix * glm::vec4(pb.x, pb.y, pa.z, 1.0f));
        addPoint(matrix * glm::vec4(pb.x, pb.y, pb.z, 1.0f));
        addPoint(matrix * glm::vec4(pa.x, pb.y, pa.z, 1.0f));
        addPoint(matrix * glm::vec4(pa.x, pa.y, pb.z, 1.0f));
        addPoint(matrix * glm::vec4(pb.x, pa.y, pb.z, 1.0f));
    }

    inline bool intersect(const AABB& aabb) {
        return (
            a.x <= aabb.b.x && b.x >= aabb.a.x && a.y <= aabb.b.y &&
            b.y >= aabb.a.y && a.z <= aabb.b.z && b.z >= aabb.a.z
        );
    }

    inline bool intersect(const AABB& aabb, float margin) {
        return (
            a.x <= aabb.b.x + margin && b.x >= aabb.a.x - margin &&
            a.y <= aabb.b.y + margin && b.y >= aabb.a.y - margin &&
            a.z <= aabb.b.z + margin && b.z >= aabb.a.z - margin
        );
    }
};
