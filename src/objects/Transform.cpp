#include "Transform.hpp"

#include "data/dv_util.hpp"

void Transform::refresh() {
    combined = glm::mat4(1.0f);
    combined = glm::translate(combined, pos);
    combined = combined * glm::mat4(rot);
    combined = glm::scale(combined, size);
    displayPos = pos;
    displaySize = size;
    dirty = false;
}

dv::value Transform::serialize() const {
    auto tsfmap = dv::object();
    tsfmap["pos"] = dv::to_value(pos);
    if (size != glm::vec3(1.0f)) {
        tsfmap["size"] = dv::to_value(size);
    }
    if (rot != glm::mat3(1.0f)) {
        tsfmap["rot"] = dv::to_value(rot);
    }
    return tsfmap;
}

void Transform::deserialize(const dv::value& root) {
    dv::get_vec(root, "pos", pos);
    dv::get_vec(root, "size", size);
    dv::get_mat(root, "rot", rot);
}
