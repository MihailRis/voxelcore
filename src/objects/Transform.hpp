#pragma once

#define GLM_ENABLE_EXPERIMENTAL

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

    inline void setRot(glm::mat3 m) {
        rot = m;
        dirty = true;
    }

    inline void setSize(glm::vec3 v) {
        if (glm::distance2(displaySize, v) >= EPSILON) {
            dirty = true;
        }
        size = v;
    }

    inline void setPos(glm::vec3 v) {
        if (glm::distance2(displayPos, v) >= EPSILON) {
            dirty = true;
        }
        pos = v;
    }
};
