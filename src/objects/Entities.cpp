#define VC_ENABLE_REFLECTION
#include "Entities.hpp"

#include "animation/rigging.hpp"
#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "data/dv_util.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "Entity.hpp"
#include "EntityDef.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/LineBatch.hpp"
#include "graphics/render/ModelBatch.hpp"
#include "logic/scripting/scripting.hpp"
#include "maths/FrustumCulling.hpp"
#include "maths/rays.hpp"
#include "maths/util.hpp"
#include "physics/PhysicsSolver.hpp"
#include "world/Level.hpp"

#include <glm/ext/matrix_transform.hpp>
#include <limits>
#include <sstream>

static debug::Logger logger("entities");

Entities::Entities(Level& level)
    : level(level),
      sensorsTickClock(20, 3),
      updateTickClock(20, 3) {
}

Entities::~Entities() = default;

void Entities::setAssets(Assets& assets) {
    this->assets = &assets;
}

Entity* Entities::get(entityid_t id) {
    const auto& found = entities.find(id);
    if (found != entities.end()) {
        return found->second.get();
    }
    return nullptr;
}

entityid_t Entities::spawn(
    const EntityDef& def,
    glm::vec3 position,
    dv::value args,
    dv::value saved,
    entityid_t uid
) {
    rigging::SkeletonConfig* skeleton = nullptr;
    if (assets) {
        skeleton = assets->get<rigging::SkeletonConfig>(def.skeletonName);
        if (skeleton == nullptr) {
            if (def.skeletonName == def.name) {
                logger.warning() << "skeleton " + def.skeletonName + " not found";
            } else {
                throw std::runtime_error("skeleton " + def.skeletonName + " not found");
            }
        }
    }
    entityid_t id;
    if (uid == 0) {
        id = nextID++;
    } else {
        id = uid;
        if (auto found = get(uid)) {
            std::stringstream ss;
            ss << "UID #" << uid << " is already used by an entity ";
            ss << found->getDef().name;
            if (found->getID().destroyFlag) {
                ss << " marked to destroy";
            }
            throw std::runtime_error(ss.str());
        }
    }
    std::unique_ptr<rigging::Skeleton> skeletonInstance;
    if (assets) {
        skeletonInstance = std::make_unique<rigging::Skeleton>(
            skeleton ? skeleton->instance() : rigging::Skeleton(nullptr)
        );
    }
    auto& entity = *(entities[id] = std::make_unique<Entity>(
        *this,
        EntityId {id, def},
        Transform {
            position, glm::vec3(1.0f), glm::mat3(1.0f), glm::mat4(1.0f), true
        },
        Rigidbody {
            true,
            Hitbox {id, def.bodyType, position, def.hitbox * 0.5f},
            std::vector<Sensor> {}
        },
        std::move(skeletonInstance)
    ));
    const auto& tsf = entity.getTransform();
    auto& body = entity.getRigidbody();
    body.initialize(def, id, *this);

    auto& scripting = entity.getScripting();

    for (auto& instance : def.components) {
        auto component = std::make_unique<UserComponent>(
            instance.component, EntityFuncsSet {}, nullptr, instance.params
        );
        scripting.components.emplace_back(std::move(component));
    }
    dv::value componentsMap = nullptr;
    if (saved != nullptr) {
        componentsMap = saved["comps"];
        loadEntity(saved, entity);
    }
    body.hitbox.position = tsf.pos;
    scripting::on_entity_spawn(
        def, id, scripting.components, args, componentsMap
    );
    return id;
}

void Entities::despawn(entityid_t id) {
    if (auto entity = get(id)) {
        auto& eid = entity->getID();
        if (!eid.destroyFlag) {
            eid.destroyFlag = true;
            scripting::on_entity_despawn(*entity);
        }
    }
}

void Entities::loadEntity(const dv::value& map) {
    entityid_t uid = map["uid"].asInteger();
    std::string defname = map["def"].asString();
    auto& def = level.content.entities.require(defname);
    spawn(def, {}, nullptr, map, uid);
}

void Entities::loadEntity(const dv::value& map, Entity& entity) {
    auto& transform = entity.getTransform();
    auto& body = entity.getRigidbody();
    auto skeleton = entity.getSkeleton();

    if (map.has(COMP_RIGIDBODY)) {
        body.deserialize(map[COMP_RIGIDBODY]);
    }
    if (map.has(COMP_TRANSFORM)) {
        transform.deserialize(map[COMP_TRANSFORM]);
    }
    if (skeleton == nullptr || skeleton->config == nullptr) {
        return;
    }
    std::string skeletonName = skeleton->config->getName();
    map.at("skeleton-name").get(skeletonName);
    if (skeletonName != skeleton->config->getName()) {
        skeleton->config = assets->getShared<rigging::SkeletonConfig>(skeletonName);
    }
    if (auto foundSkeleton = map.at(COMP_SKELETON)) {
        skeleton->deserialize(*foundSkeleton);
    }
}

std::optional<Entities::RaycastResult> Entities::rayCast(
    glm::vec3 start,
    glm::vec3 dir,
    float maxDistance,
    const RaycastSettings& settings
) {
    Ray ray(start, dir);
    entityid_t foundUID = 0;
    glm::ivec3 foundNormal;

    for (const auto& [uid, entity] : entities) {
        const auto& eid = entity->getID();
        const auto& body = entity->getRigidbody();
        const auto& hitbox = body.hitbox;
        if (eid.uid == settings.ignoredUid || !body.enabled ||
            (settings.solidEntitiesOnly && !eid.def.solid) ||
            (!hitbox.selectable && !settings.includeNonSelectable)) {
            continue;
        }
        if (settings.entitiesFilter) {
            bool matches = settings.entitiesFilter->find(eid.def.rt.id) !=
                           settings.entitiesFilter->end();
            if (matches == settings.entityFilterExcludeMode) {
                continue;
            }
        }
        glm::ivec3 normal;
        double distance;
        if (ray.intersectAABB(
                glm::vec3(), hitbox.getAABB(), maxDistance, normal, distance
            ) > RayRelation::None) {
            foundUID = eid.uid;
            foundNormal = normal;
            maxDistance = static_cast<float>(distance);
        }
    }
    if (foundUID) {
        return Entities::RaycastResult {foundUID, foundNormal, maxDistance};
    } else {
        return std::nullopt;
    }
}

void Entities::loadEntities(dv::value root) {
    clean();
    const auto& list = root["data"];
    for (auto& map : list) {
        try {
            loadEntity(map);
        } catch (const std::runtime_error& err) {
            logger.error() << "could not read entity: " << err.what();
        }
    }
}

void Entities::onSave(const Entity& entity) {
    scripting::on_entity_save(entity);
}

dv::value Entities::serialize(const std::vector<Entity*>& entities) {
    auto list = dv::list();
    for (auto entity : entities) {
        const EntityId& eid = entity->getID();
        if (!entity->getDef().save.enabled || eid.destroyFlag) {
            continue;
        }
        level.entities->onSave(*entity);
        if (!eid.destroyFlag) {
            list.add(entity->serialize());
        }
    }
    return list;
}

void Entities::despawn(const std::vector<Entity*>& entities) {
    for (auto entity : entities) {
        entity->destroy();
    }
}

void Entities::clean() {
    for (auto it = entities.begin(); it != entities.end();) {
        if (!it->second->getID().destroyFlag) {
            ++it;
        } else {
            auto& rigidbody = it->second->getRigidbody();
            // todo: refactor
            auto physics = level.physics.get();
            for (auto& sensor : rigidbody.sensors) {
                physics->removeSensor(&sensor);
            }
            it = entities.erase(it);
        }
    }
}

void Entities::updateSensors(
    Rigidbody& body, const Transform& tsf, std::vector<Sensor*>& sensors
) {
    for (size_t i = 0; i < body.sensors.size(); i++) {
        auto& sensor = body.sensors[i];
        for (auto oid : sensor.prevEntered) {
            if (sensor.nextEntered.find(oid) == sensor.nextEntered.end()) {
                sensor.exitCallback(sensor.entity, i, oid);
            }
        }
        sensor.prevEntered = sensor.nextEntered;
        sensor.nextEntered.clear();

        switch (sensor.type) {
            case SensorType::AABB:
                sensor.calculated.aabb = sensor.params.aabb;
                sensor.calculated.aabb.transform(tsf.combined);
                break;
            case SensorType::RADIUS:
                sensor.calculated.radial = glm::vec4(
                    body.hitbox.position.x,
                    body.hitbox.position.y,
                    body.hitbox.position.z,
                    sensor.params.radial.w * sensor.params.radial.w
                );
                break;
        }
        sensors.push_back(&sensor);
    }
}

void Entities::preparePhysics(float delta) {
    auto& physics = *level.physics;
    auto& hitboxes = physics.getHitboxesWriteable();
    auto& solidHitboxes = physics.getSolidHitboxesWriteable();
    auto& sensors = physics.getSensorsWriteable();
    sensors.clear();

    if (int parts = sensorsTickClock.update(delta)) {
        for (int i = 0; i < parts; i++) {
            auto part = sensorsTickClock.convertPart(i);
            auto allParts = sensorsTickClock.getParts();

            for (const auto& [uid, entity] : entities) {
                const auto& eid = entity->getID();
                auto& rigidbody = entity->getRigidbody();
                if (!rigidbody.enabled) {
                    continue;
                }
                if ((eid.uid + part) % allParts != 0) {
                    continue;
                }
                updateSensors(rigidbody, entity->getTransform(), sensors);
            }
        }
    }

    hitboxes.clear();
    solidHitboxes.clear();

    for (const auto& [uid, entity] : entities) {
        const auto& eid = entity->getID();
        auto& rigidbody = entity->getRigidbody();
        auto bodyType = rigidbody.hitbox.type;
        if (eid.destroyFlag || !rigidbody.enabled) {
            continue;
        }
        rigidbody.hitbox.selectable = rigidbody.selectable;
        rigidbody.hitbox.mass = bodyType == BodyType::DYNAMIC
                            ? rigidbody.mass
                            : std::numeric_limits<float>::infinity();
        rigidbody.hitbox.elasticity = rigidbody.elasticity;
        if (bodyType == BodyType::STATIC) {
            continue;
        }
        hitboxes.emplace_back(&rigidbody.hitbox);
        if (!eid.def.solid) {
            continue;
        }
        solidHitboxes.emplace_back(&rigidbody.hitbox);
    }
}

void Entities::updatePhysics(float delta) {
    preparePhysics(delta);

    auto physics = level.physics.get();

    int substeps = std::max<int>(std::min<int>(delta * 1000, 200), 8);
    physics->step(*level.chunks, delta, substeps);

    for (const auto& [uid, entity] : entities) {
        const auto& eid = entity->getID();
        auto& transform = entity->getTransform();
        auto& rigidbody = entity->getRigidbody();
        if (!rigidbody.enabled ||
            rigidbody.hitbox.type == BodyType::STATIC) {
            continue;
        }
        auto& hitbox = rigidbody.hitbox;
        hitbox.scale = transform.size;
        if (util::is_nan_or_inf(hitbox.position)) {
            logger.error()
                << "physics simulation produced nan or inf (entity "
                << eid.def.name << "#" << eid.uid << ")";
            hitbox.position = transform.pos;
        } else {
            transform.setPos(hitbox.position);
        }
        if (hitbox.grounded && !hitbox.prevGrounded) {
            scripting::on_entity_grounded(
                *entity, glm::length(hitbox.prevVelocity - hitbox.velocity)
            );
        }
        if (!hitbox.grounded && hitbox.prevGrounded) {
            scripting::on_entity_fall(*entity);
        }
    }
}

void Entities::update(float delta) {
    if (int parts = updateTickClock.update(delta)) {
        for (int i = 0; i < parts; i++) {
            scripting::on_entities_update(
                updateTickClock.getTickRate(),
                updateTickClock.getParts(),
                updateTickClock.convertPart(i)
            );
        }
    }
    updatePhysics(delta);
    scripting::on_entities_physics_update(delta);

    for (const auto& [uid, entity] : entities) {
        if (!entity->getSkeleton()) {
            continue;
        }
        auto& transform = entity->getTransform();
        auto& skeleton = *entity->getSkeleton();
        if (transform.dirty) {
            transform.refresh();
        }
        if (skeleton.interpolation.isEnabled()) {
            skeleton.interpolation.updateTimer(delta);
        }
        if (skeleton.rotationInterpolation.isEnabled()) {
            skeleton.rotationInterpolation.updateTimer(delta);
        }
    }
}

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

void Entities::renderDebug(
    LineBatch& batch, const Frustum* frustum, const DrawContext& pctx, entityid_t fpsEntity
) {
    {
        auto ctx = pctx.sub(&batch);
        ctx.setLineWidth(1);
        for (const auto& [uid, entity] : entities) {
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
        for (const auto& [uid, entity] : entities) {
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

void Entities::render(
    const Assets& assets,
    ModelBatch& batch,
    const Frustum* frustum,
    entityid_t fpsEntity
) {
    for (const auto& [uid, entity] : entities) {
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

void Entities::updateCulling(const Frustum* frustum, entityid_t fpsEntity) {
    for (const auto& [uid, entity] : entities) {
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

bool Entities::hasBlockingInside(AABB aabb) {
    constexpr float eps = 0.05f;
    for (const auto& [uid, entity] : entities) {
        AABB bodyAABB(entity->getRigidbody().hitbox.getAABB());
        bodyAABB.scale(glm::vec3(1.0f - eps));
        if (entity->getID().def.blocking && aabb.intersects(bodyAABB)) {
            return true;
        }
    }
    return false;
}

std::vector<Entity*> Entities::getAllInside(AABB aabb) {
    std::vector<Entity*> collected;
    for (const auto& [uid, entity] : entities) {
        if (!entity->getID().destroyFlag && aabb.contains(entity->getTransform().pos)) {
            collected.push_back(entity.get());
        }
    }
    return collected;
}

std::vector<Entity*> Entities::getAllInRadius(glm::vec3 center, float radius) {
    std::vector<Entity*> collected;
    for (const auto& [uid, entity] : entities) {
        if (glm::distance2(entity->getTransform().pos, center) <= radius * radius) {
            collected.push_back(entity.get());
        }
    }
    return collected;
}
