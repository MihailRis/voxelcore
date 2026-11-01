#include "EntitiesRenderer.hpp"

#include "animation/rigging.hpp"
#include "assets/Assets.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/LineBatch.hpp"
#include "maths/FrustumCulling.hpp"
#include "objects/Entities.hpp"
#include "objects/Entity.hpp"
#include "objects/EntityDef.hpp"
#include "ModelBatch.hpp"

static void debug_render_skeleton(
    LineBatch& batch,
    const rigging::Bone* bone,
    const rigging::Skeleton& skeleton
) {
    size_t pindex = bone->getIndex();
    for (auto& sub : bone->getBones()) {
        size_t sindex = sub->getIndex();
        const auto& matrices = skeleton.calculated.matrices;
        batch.line(
            glm::vec3(matrices[pindex] * glm::vec4(0, 0, 0, 1)),
            glm::vec3(matrices[sindex] * glm::vec4(0, 0, 0, 1)),
            glm::vec4(0, 0.5f, 0, 1)
        );
        debug_render_skeleton(batch, sub.get(), skeleton);
    }
}

static glm::vec3 calc_extent(
    const Transform& transform,
    const Rigidbody& rigidbody,
    const rigging::Skeleton& skeleton
) {
    const auto& size = transform.size;
    float scale = glm::max(
        glm::abs(size.x), glm::max(glm::abs(size.y), glm::abs(size.z))
    );
    return glm::max(
        rigidbody.hitbox.getHalfSize(), glm::vec3(skeleton.boundsRadius * scale)
    );
}

static bool check_frustum(
    const Frustum& frustum,
    const Transform& transform,
    const Rigidbody& rigidbody,
    const rigging::Skeleton& skeleton
) {
    glm::vec3 ext = calc_extent(transform, rigidbody, skeleton);
    return frustum.isBoxVisible(transform.pos - ext, transform.pos + ext);
}

EntitiesRenderer::EntitiesRenderer(const Assets& assets, Entities& entities)
    : assets(assets), entities(entities) {
}

void EntitiesRenderer::render(
    ModelBatch& batch, const Frustum* frustum, entityid_t fpsEntity
) {
    for (const auto& [uid, entity] : entities.getEntities()) {
        if (!entity->getSkeleton() || uid == fpsEntity) {
            continue;
        }
        const auto& transform = entity->getTransform();
        const auto& rigidbody = entity->getRigidbody();
        auto& skeleton = *entity->getSkeleton();
        const auto& def = entity->getID().def;
        const auto& pos = transform.pos;
        const auto& size = transform.size;
        const auto& rigConfig = skeleton.config;
        if (rigConfig == nullptr) {
            continue;
        }
        if (frustum && !check_frustum(*frustum, transform, rigidbody, skeleton)) {
            if (!skeleton.boundsDirty || !skeleton.visible) {
                continue;
            }
            rigConfig->updateBounds(assets, skeleton, transform.rot, pos, size);
            if (!check_frustum(*frustum, transform, rigidbody, skeleton)) {
                continue;
            }
        }
        rigConfig->render(
            assets,
            batch,
            skeleton,
            def.lightingMode,
            transform.rot,
            pos,
            size,
            def.translucent
        );
    }
}

void EntitiesRenderer::renderDebug(
    LineBatch& batch,
    const Frustum* frustum,
    const DrawContext& pctx,
    entityid_t fpsEntity
) {
    const auto& all = entities.getEntities();
    {
        auto ctx = pctx.sub(&batch);
        ctx.setLineWidth(1);
        for (const auto& [uid, entity] : all) {
            if (!entity->getSkeleton()) {
                continue;
            }
            const auto& transform = entity->getTransform();
            const auto& rigidbody = entity->getRigidbody();
            const auto& skeleton = *entity->getSkeleton();
            const auto& hitbox = rigidbody.hitbox;
            if (frustum && !check_frustum(*frustum, transform, rigidbody, skeleton)) {
                continue;
            }
            batch.box(
                hitbox.position, hitbox.getHalfSize() * 2.0f, glm::vec4(1.0f)
            );
            glm::vec3 ext = calc_extent(transform, rigidbody, skeleton);
            batch.box(
                transform.pos, ext * 2.0f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)
            );
            for (auto& sensor : rigidbody.sensors) {
                if (sensor.type != SensorType::AABB) continue;
                batch.box(
                    sensor.calculated.aabb.center(),
                    sensor.calculated.aabb.size(),
                    glm::vec4(1.0f, 1.0f, 0.0f, 1.0f)
                );
            }
        }
    }
    {
        auto ctx = pctx.sub(&batch);
        ctx.setDepthTest(false);
        ctx.setDepthMask(false);
        ctx.setLineWidth(2);
        for (const auto& [uid, entity] : all) {
            if (!entity->getSkeleton() || uid == fpsEntity) {
                continue;
            }
            const auto& transform = entity->getTransform();
            const auto& rigidbody = entity->getRigidbody();
            const auto& skeleton = *entity->getSkeleton();
            auto config = skeleton.config;
            if (config == nullptr) {
                continue;
            }
            if (frustum && !check_frustum(*frustum, transform, rigidbody, skeleton)) {
                continue;
            }
            auto bone = config->getRoot();
            debug_render_skeleton(batch, bone, skeleton);
        }
    }
}

void EntitiesRenderer::updateCulling(
    const Frustum* frustum, entityid_t fpsEntity
) {
    for (const auto& [uid, entity] : entities.getEntities()) {
        if (!entity->getSkeleton()) {
            continue;
        }
        auto& skeleton = *entity->getSkeleton();
        if (frustum == nullptr || uid == fpsEntity) {
            skeleton.culled = false;
            continue;
        }
        skeleton.culled = !check_frustum(
            *frustum, entity->getTransform(), entity->getRigidbody(), skeleton
        );
    }
}
