#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <stdexcept>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/norm.hpp>
#include <data/dv_fwd.hpp>

struct Transform {
    static inline constexpr float EPSILON = 1e-7f;
    glm::vec3 pos;
    glm::vec3 size;
    glm::mat3 rot;
    glm::mat4 combined;
    bool dirty = true;

    glm::vec3 displayPos;
    glm::vec3 displaySize;

    dv::value serialize() const;
    void deserialize(const dv::value& root);

    void refresh();

    inline void setRot(const glm::mat3& m) {
        if (!checkValue(m, "rotation")) {
            return;
        }
        rot = m;
        dirty = true;
    }

    inline void setSize(const glm::vec3& v) {
        if (!checkValue(v, "size")) {
            return;
        }
        if (glm::distance2(displaySize, v) >= EPSILON) {
            dirty = true;
        }
        size = v;
    }

    inline void setPos(const glm::vec3& v) {
        if (!checkValue(v, "position")) {
            return;
        }
        if (glm::distance2(displayPos, v) >= EPSILON) {
            dirty = true;
        }
        pos = v;
    }

    static bool checkValue(const glm::vec3& v, std::string_view name);
    static bool checkValue(const glm::mat3& v, std::string_view name);
};
