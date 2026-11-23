#pragma once

#include "typedefs.hpp"

#include <glm/vec3.hpp>

class Assets;
class DrawContext;
class Entities;
class Frustum;
class LineBatch;
class ModelBatch;

class EntitiesRenderer {
    const Assets& assets;
    Entities& entities;
public:
    EntitiesRenderer(const Assets& assets, Entities& entities);

    void render(
        ModelBatch& batch,
        const glm::ivec3& origin,
        const Frustum* frustum,
        entityid_t fpsEntity
    );

    void renderDebug(
        LineBatch& batch,
        const glm::ivec3& origin,
        const Frustum* frustum,
        const DrawContext& ctx,
        entityid_t fpsEntity
    );

    void updateCulling(
        const glm::ivec3& origin, const Frustum* frustum, entityid_t fpsEntity
    );
};
