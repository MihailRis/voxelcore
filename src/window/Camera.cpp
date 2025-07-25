#include "Camera.hpp"

#include <cmath>
#include <glm/ext.hpp>

Camera::Camera(glm::vec3 position, float fov) : fov(fov), position(position) {
    updateVectors();
}

void Camera::updateVectors() {
    front = glm::vec3(rotation * glm::vec4(0, 0, -1, 1));
    right = glm::vec3(rotation * glm::vec4(1, 0, 0, 1));
    up = glm::vec3(rotation * glm::vec4(0, 1, 0, 1));
    dir = glm::vec3(rotation * glm::vec4(0, 0, -1, 1));
    dir.y = 0;
    float len = glm::length(dir);
    if (len > 0.0f) {
        dir.x /= len;
        dir.z /= len;
    }
}

void Camera::rotate(float x, float y, float z) {
    rotation = glm::rotate(rotation, y, glm::vec3(0, 1, 0));
    rotation = glm::rotate(rotation, x, glm::vec3(1, 0, 0));
    rotation = glm::rotate(rotation, z, glm::vec3(0, 0, 1));
    updateVectors();
}

glm::mat4 Camera::getProjection() const {
    if (projset) {
        return projection;
    }
    if (perspective) {
        return glm::perspective(fov * zoom, ar, near, far);
    } else if (flipped) {
        return glm::ortho(0.0f, fov * ar, fov, 0.0f, near, far);
    } else {
        return glm::ortho(0.0f, fov * ar, 0.0f, fov, near, far);
    }
}

glm::mat4 Camera::getView(bool pos) const {
    glm::vec3 camera_pos = this->position;
    if (!pos) {
        camera_pos = glm::vec3(0.0f);
    }
    if (perspective) {
        return glm::lookAt(camera_pos, camera_pos + front, up);
    } else {
        return glm::lookAt(camera_pos, camera_pos + front, up);
        //return glm::translate(glm::mat4(1.0f), camera_pos);
    }
}

glm::mat4 Camera::getProjView(bool pos) const {
    return getProjection() * getView(pos);
}

void Camera::setFov(float fov) {
    this->fov = fov;
}

float Camera::getFov() const {
    return fov;
}

void Camera::setProjection(const glm::mat4& matrix) {
    projection = matrix;
    projset = true;
}

float Camera::getAspectRatio() const {
    return ar;
}

void Camera::setAspectRatio(float ar) {
    this->ar = ar;
}
