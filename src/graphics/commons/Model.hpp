#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "maths/UVRegion.hpp"

namespace model {
    struct Vertex {
        glm::vec3 coord;
        glm::vec2 uv;
        glm::vec3 normal;
    };

    struct Mesh {
        std::string texture;
        std::vector<Vertex> vertices;
        bool shading = true;

        void addPlane(
            const glm::vec3& pos,
            const glm::vec3& right,
            const glm::vec3& up,
            const glm::vec3& norm
        );
        void addPlane(
            const glm::vec3& pos,
            const glm::vec3& right,
            const glm::vec3& up,
            const glm::vec3& norm,
            const UVRegion& region
        );
        void addRect(
            const glm::vec3& pos,
            const glm::vec3& right,
            const glm::vec3& up,
            const glm::vec3& norm,
            const UVRegion& region
        );
        void addBox(const glm::vec3& pos, const glm::vec3& size);
        void addBox(
            const glm::vec3& pos,
            const glm::vec3& size,
            const UVRegion (&texfaces)[6],
            const bool enabledSides[6]
        );
        void scale(const glm::vec3& size);
    };

    struct Model {
        std::vector<Mesh> meshes;

        /// @brief Add mesh to the model
        /// @param texture texture name
        /// @return writeable Mesh
        Mesh& addMesh(const std::string& texture, bool shading = true) {
            for (auto& mesh : meshes) {
                if (mesh.texture == texture && mesh.shading == shading) {
                    return mesh;
                }
            }
            meshes.push_back({texture, {}, shading});
            return meshes[meshes.size() - 1];
        }
        /// @brief Remove all empty meshes
        void clean();
    };
}
