#pragma once

#include "typedefs.hpp"

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

class Camera {
    float fov = 1.0f;
    float ar = 0.0f;
public:
    glm::vec3 front {};
    glm::vec3 up {};
    glm::vec3 right {};
    glm::vec3 dir {};

    worldvec3 position {};

    float zoom = 1.0f;
    glm::mat4 rotation {1.0f};
    bool perspective = true;
    bool flipped = false;
    float near = 0.05f;
    float far = 1e4f;

    bool projset = false;
    glm::mat4 projection;

    Camera() {
        updateVectors();
    }
    Camera(worldvec3 position, float fov);

    void updateVectors();
    void rotate(float x, float y, float z);

    /// @brief Get origin of the rendering coordinate system: an integer point
    /// near the camera, getView() is relative to it
    glm::ivec3 getOrigin() const;
    glm::vec3 getLocalPosition() const;

    glm::mat4 getProjection() const;
    glm::mat4 getView(bool position = true) const;
    glm::mat4 getProjView(bool position = true) const;

    void setFov(float fov);
    float getFov() const;

    void setProjection(const glm::mat4& matrix);

    float getAspectRatio() const;
    void setAspectRatio(float ar);
};
