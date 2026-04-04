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
            const glm::vec3& norm,
            const UVRegion& region
        );
        void addPlane(
            const glm::vec3& pos,
            const glm::vec3& right,
            const glm::vec3& up,
            const glm::vec3& norm,
            const UVRegion& region,
            const glm::mat4& transform
        );
        void addRect(
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
            const UVRegion& uv,
            const glm::mat4& transform
        );
        void addTriangle(
            const glm::vec3& a,
            const glm::vec3& b,
            const glm::vec3& c,
            const glm::vec3& norm,
            const glm::vec2& uvA,
            const glm::vec2& uvB,
            const glm::vec2& uvC
        );
        void addBox(const glm::vec3& pos, const glm::vec3& size);
        void addBox(
            const glm::vec3& pos,
            const glm::vec3& size,
            const UVRegion (&texfaces)[6],
            const bool enabledSides[6]
        );
        void addBox(
            const glm::vec3& pos,
            const glm::vec3& size,
            const UVRegion (&texfaces)[6],
            const bool enabledSides[6],
            const glm::mat4& transform
        );
        void scale(const glm::vec3& size);

        /// @brief Apply transformation matrix to vertices coordinates
        void transform(const glm::mat4& matrix);

        /// @brief Move vertices coordinates by offset
        void translate(const glm::vec3& offset);
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

        /// @brief Apply transformation matrix to vertices coordinates
        void transform(const glm::mat4& matrix);

        /// @brief Move vertices coordinates by offset
        void translate(const glm::vec3& offset);

        /// @brief Copy or merge meshes from source model to this.
        /// Does not remove duplicates
        void merge(const Model& source);

        /// @brief Move or merge meshes from source model to this.
        /// Does not remove duplicates
        void merge(Model&& source);
    };
}
