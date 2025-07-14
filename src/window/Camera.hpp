#pragma once

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

    glm::vec3 position {};

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
    Camera(glm::vec3 position, float fov);

    void updateVectors();
    void rotate(float x, float y, float z);

    glm::mat4 getProjection() const;
    glm::mat4 getView(bool position = true) const;
    glm::mat4 getProjView(bool position = true) const;

    void setFov(float fov);
    float getFov() const;

    void setProjection(const glm::mat4& matrix);

    float getAspectRatio() const;
    void setAspectRatio(float ar);
};
