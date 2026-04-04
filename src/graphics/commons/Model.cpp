#define GLM_ENABLE_EXPERIMENTAL

#include "Model.hpp"

#include <algorithm>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>

using namespace model;

inline constexpr glm::vec3 X(1, 0, 0);
inline constexpr glm::vec3 Y(0, 1, 0);
inline constexpr glm::vec3 Z(0, 0, 1);

void Mesh::addPlane(
    const glm::vec3& pos,
    const glm::vec3& right,
    const glm::vec3& up,
    const glm::vec3& norm,
    const UVRegion& uv
) {
    vertices.push_back({pos-right-up, {uv.u1, uv.v1}, norm});
    vertices.push_back({pos+right-up, {uv.u2, uv.v1}, norm});
    vertices.push_back({pos+right+up, {uv.u2, uv.v2}, norm});

    vertices.push_back({pos-right-up, {uv.u1, uv.v1}, norm});
    vertices.push_back({pos+right+up, {uv.u2, uv.v2}, norm});
    vertices.push_back({pos-right+up, {uv.u1, uv.v2}, norm});
}

void Mesh::addPlane(
    const glm::vec3& pos,
    const glm::vec3& right,
    const glm::vec3& up,
    const glm::vec3& norm,
    const UVRegion& region,
    const glm::mat4& transform
) {
    addPlane(
        glm::vec3(transform * glm::vec4(pos, 1.0f)),
        glm::vec3(transform * glm::vec4(right, 0.0f)),
        glm::vec3(transform * glm::vec4(up, 0.0f)),
        glm::normalize(glm::vec3(transform * glm::vec4(norm, 0.0f))),
        region
    );
}

void Mesh::addTriangle(
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& c,
    const glm::vec3& norm,
    const glm::vec2& uvA,
    const glm::vec2& uvB,
    const glm::vec2& uvC
) {
    vertices.push_back({a, uvA, norm});
    vertices.push_back({b, uvB, norm});
    vertices.push_back({c, uvC, norm});
}

void Mesh::addRect(
    const glm::vec3& pos,
    const glm::vec3& right,
    const glm::vec3& up,
    const glm::vec3& norm,
    const UVRegion& uv
) {
    vertices.push_back({pos-right-up, {uv.u1, uv.v1}, norm});
    vertices.push_back({pos+right-up, {uv.u2, uv.v1}, norm});
    vertices.push_back({pos+right+up, {uv.u2, uv.v2}, norm});

    vertices.push_back({pos-right-up, {uv.u1, uv.v1}, norm});
    vertices.push_back({pos+right+up, {uv.u2, uv.v2}, norm});
    vertices.push_back({pos-right+up, {uv.u1, uv.v2}, norm});
}

static glm::mat4 extract_rotation(const glm::mat4& matrix) {
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(matrix, scale, rotation, translation, skew, perspective);
    return glm::mat4_cast(rotation);
}

void Mesh::addRect(
    const glm::vec3& pos,
    const glm::vec3& right,
    const glm::vec3& up,
    const glm::vec3& norm,
    const UVRegion& uv,
    const glm::mat4& transform
) {
    auto a = transform * glm::vec4(pos - right - up, 1.0f);
    auto b = transform * glm::vec4(pos + right - up, 1.0f);
    auto c = transform * glm::vec4(pos + right + up, 1.0f);
    auto d = transform * glm::vec4(pos - right + up, 1.0f);
    auto tsfnorm = extract_rotation(transform) * glm::vec4(norm, 1.0f);

    vertices.push_back({a, {uv.u1, uv.v1}, tsfnorm});
    vertices.push_back({b, {uv.u2, uv.v1}, tsfnorm});
    vertices.push_back({c, {uv.u2, uv.v2}, tsfnorm});

    vertices.push_back({a, {uv.u1, uv.v1}, tsfnorm});
    vertices.push_back({c, {uv.u2, uv.v2}, tsfnorm});
    vertices.push_back({d, {uv.u1, uv.v2}, tsfnorm});
}

void Mesh::addBox(const glm::vec3& pos, const glm::vec3& size) {
    UVRegion fullRegion (0, 0, 1, 1);
    addPlane(pos+Z*size, X*size, Y*size, Z, fullRegion);
    addPlane(pos-Z*size, -X*size, Y*size, -Z, fullRegion);

    addPlane(pos+Y*size, X*size, -Z*size, Y, fullRegion);
    addPlane(pos-Y*size, X*size, Z*size, -Y, fullRegion);

    addPlane(pos+X*size, -Z*size, Y*size, X, fullRegion);
    addPlane(pos-X*size, Z*size, Y*size, -X, fullRegion);
}

void Mesh::addBox(
    const glm::vec3& pos,
    const glm::vec3& size,
    const UVRegion (&uvs)[6],
    const bool enabledSides[6]
) {
    addBox(pos, size, uvs, enabledSides, glm::mat4(1.0f));
}

void Mesh::addBox(
    const glm::vec3& pos,
    const glm::vec3& size,
    const UVRegion (&uvs)[6],
    const bool enabledSides[6],
    const glm::mat4& transform
) {
    if (enabledSides[0]) // north
        addPlane(pos+Z*size, X*size, Y*size, Z, uvs[0], transform);
    if (enabledSides[1]) // south
        addPlane(pos-Z*size, -X*size, Y*size, -Z, uvs[1], transform);
    if (enabledSides[2]) // top
        addPlane(pos+Y*size, X*size, -Z*size, Y, uvs[2] * glm::vec2(-1), transform);
    if (enabledSides[3]) // bottom
        addPlane(pos-Y*size, X*size, Z*size, -Y, uvs[3] * glm::vec2(-1, 1), transform);
    if (enabledSides[4]) // west
        addPlane(pos+X*size, -Z*size, Y*size, X, uvs[4], transform);
    if (enabledSides[5]) // east
        addPlane(pos-X*size, Z*size, Y*size, -X, uvs[5] * glm::vec2(-1, 1), transform);
}

void Mesh::scale(const glm::vec3& size) {
    for (auto& vertex : vertices) {
        vertex.coord *= size;
    }
}

void Mesh::transform(const glm::mat4& matrix) {
    for (auto& vertex : vertices) {
        vertex.coord = matrix * glm::vec4(vertex.coord, 1.0f);
    }
}

void Mesh::translate(const glm::vec3& offset) {
    for (auto& vertex : vertices) {
        vertex.coord += offset;
    }
}

void Model::clean() {
    meshes.erase(
        std::remove_if(meshes.begin(), meshes.end(), 
        [](const Mesh& mesh){
            return mesh.vertices.empty();
        }),
        meshes.end()
    );
}

void Model::transform(const glm::mat4& matrix) {
    for (auto& mesh : meshes) {
        mesh.transform(matrix);
    }
}

void Model::translate(const glm::vec3& offset) {
    for (auto& mesh : meshes) {
        mesh.translate(offset);
    }
}

void Model::merge(const Model& source) {
    for (const auto& srcMesh : source.meshes) {
        auto& dstMesh = addMesh(srcMesh.texture, srcMesh.shading);
        dstMesh.vertices.insert(
            dstMesh.vertices.end(),
            srcMesh.vertices.begin(),
            srcMesh.vertices.end()
        );
    }
}

void Model::merge(Model&& source) {
    for (auto& srcMesh : source.meshes) {
        auto& dstMesh = addMesh(srcMesh.texture, srcMesh.shading);
        if (dstMesh.vertices.empty()) {
            dstMesh = std::move(srcMesh);
        } else {
            dstMesh.vertices.insert(
                dstMesh.vertices.end(),
                srcMesh.vertices.begin(),
                srcMesh.vertices.end()
            );
        }
    }
}
