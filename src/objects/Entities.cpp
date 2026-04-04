#define VC_ENABLE_REFLECTION
#include "Entities.hpp"

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
#include "rigging.hpp"
#include "world/Level.hpp"

#include <entt/entity/registry.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <limits>
#include <sstream>

static debug::Logger logger("entities");

Entities::Entities(Level& level)
    : registry(std::make_unique<entt::registry>()),
      level(level),
      sensorsTickClock(20, 3),
      updateTickClock(20, 3) {
}

Entities::~Entities() = default;

void Entities::setAssets(Assets& assets) {
    this->assets = &assets;
}

std::optional<Entity> Entities::get(entityid_t id) {
    const auto& found = entities.find(id);
    if (found != entities.end() && registry->valid(found->second)) {
        return Entity(*this, id, *registry, found->second);
    }
    return std::nullopt;
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
            throw std::runtime_error("skeleton " + def.skeletonName + " not found");
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
    auto entity = registry->create();
    entities[id] = entity;
    uids[entity] = id;

    registry->emplace<EntityId>(entity, id, def);
    const auto& tsf = registry->emplace<Transform>(
        entity,
        position,
        glm::vec3(1.0f),
        glm::mat3(1.0f),
        glm::mat4(1.0f),
        true
    );
    auto& body = registry->emplace<Rigidbody>(
        entity,
        true,
        Hitbox {id, def.bodyType, position, def.hitbox * 0.5f},
        std::vector<Sensor> {}
    );
    body.initialize(def, id, *this);

    auto& scripting = registry->emplace<ScriptComponents>(entity);
    if (assets) {
        registry->emplace<rigging::Skeleton>(entity, skeleton->instance());
    }

    for (auto& instance : def.components) {
        auto component = std::make_unique<UserComponent>(
            instance.component, EntityFuncsSet {}, nullptr, instance.params
        );
        scripting.components.emplace_back(std::move(component));
    }
    dv::value componentsMap = nullptr;
    if (saved != nullptr) {
        componentsMap = saved["comps"];
        loadEntity(saved, get(id).value());
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

void Entities::loadEntity(const dv::value& map, Entity entity) {
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
        skeleton->config = assets->get<rigging::SkeletonConfig>(skeletonName);
    }
    if (auto foundSkeleton = map.at(COMP_SKELETON)) {
        skeleton->deserialize(*foundSkeleton);
    }
}

std::optional<Entities::RaycastResult> Entities::rayCast(
    glm::vec3 start,
    glm::vec3 dir,
    float maxDistance,
    entityid_t ignore,
    bool solidOnly
) {
    Ray ray(start, dir);
    auto view = registry->view<EntityId, Transform, Rigidbody>();

    entityid_t foundUID = 0;
    glm::ivec3 foundNormal;

    for (auto [entity, eid, transform, body] : view.each()) {
        if (eid.uid == ignore || !body.enabled || (solidOnly && !eid.def.solid)) {
            continue;
        }
        auto& hitbox = body.hitbox;
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

dv::value Entities::serialize(const std::vector<Entity>& entities) {
    auto list = dv::list();
    for (auto& entity : entities) {
        const EntityId& eid = entity.getID();
        if (!entity.getDef().save.enabled || eid.destroyFlag) {
            continue;
        }
        level.entities->onSave(entity);
        if (!eid.destroyFlag) {
            list.add(entity.serialize());
        }
    }
    return list;
}

void Entities::despawn(std::vector<Entity> entities) {
    for (auto& entity : entities) {
        entity.destroy();
    }
}

void Entities::clean() {
    for (auto it = entities.begin(); it != entities.end();) {
        if (!registry->get<EntityId>(it->second).destroyFlag) {
            ++it;
        } else {
            auto& rigidbody = registry->get<Rigidbody>(it->second);
            // todo: refactor
            auto physics = level.physics.get();
            for (auto& sensor : rigidbody.sensors) {
                physics->removeSensor(&sensor);
            }
            uids.erase(it->second);
            registry->destroy(it->second);
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

    if (sensorsTickClock.update(delta)) {
        auto part = sensorsTickClock.getPart();
        auto parts = sensorsTickClock.getParts();

        auto& sensors = physics.getSensorsWriteable();
        sensors.clear();

        auto view = registry->view<EntityId, Transform, Rigidbody>();
        for (auto [entity, eid, transform, rigidbody] : view.each()) {
            if (!rigidbody.enabled) {
                continue;
            }
            if ((eid.uid + part) % parts != 0) {
                continue;
            }
            updateSensors(rigidbody, transform, sensors);
        }
    }

    hitboxes.clear();
    solidHitboxes.clear();

    auto view = registry->view<EntityId, Rigidbody>();
    for (auto [entity, eid, rigidbody] : view.each()) {
        auto bodyType = rigidbody.hitbox.type;
        if (eid.destroyFlag || !rigidbody.enabled || bodyType == BodyType::STATIC) {
            continue;
        }
        rigidbody.hitbox.mass = bodyType == BodyType::DYNAMIC
                            ? rigidbody.mass
                            : std::numeric_limits<float>::infinity();
        rigidbody.hitbox.elasticity = rigidbody.elasticity;
        hitboxes.emplace_back(&rigidbody.hitbox);
        if (!eid.def.solid) {
            continue;
        }
        solidHitboxes.emplace_back(&rigidbody.hitbox);
    }
}

void Entities::updatePhysics(float delta) {
    preparePhysics(delta);

    auto view = registry->view<EntityId, Transform, Rigidbody>();
    auto physics = level.physics.get();

    int substeps = std::max<int>(std::min<int>(delta * 1000, 200), 8);
    physics->step(*level.chunks, delta, substeps);

    for (auto [entity, eid, transform, rigidbody] : view.each()) {
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
                *get(eid.uid), glm::length(hitbox.prevVelocity - hitbox.velocity)
            );
        }
        if (!hitbox.grounded && hitbox.prevGrounded) {
            scripting::on_entity_fall(*get(eid.uid));
        }
    }
}

void Entities::update(float delta) {
    if (updateTickClock.update(delta)) {
        scripting::on_entities_update(
            updateTickClock.getTickRate(),
            updateTickClock.getParts(),
            updateTickClock.getPart()
        );
    }
    updatePhysics(delta);
    scripting::on_entities_physics_update(delta);

    auto view = registry->view<Transform, rigging::Skeleton>();
    for (auto [entity, transform, skeleton] : view.each()) {
        if (transform.dirty) {
            transform.refresh();
        }
        if (skeleton.interpolation.isEnabled()) {
            skeleton.interpolation.updateTimer(delta);
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

void Entities::renderDebug(
    LineBatch& batch, const Frustum* frustum, const DrawContext& pctx
) {
    {
        auto ctx = pctx.sub(&batch);
        ctx.setLineWidth(1);
        auto view = registry->view<Transform, Rigidbody>();
        for (auto [entity, transform, rigidbody] : view.each()) {
            const auto& hitbox = rigidbody.hitbox;
            const auto& pos = transform.pos;
            const auto& size = transform.size;
            if (frustum && !frustum->isBoxVisible(pos - size, pos + size)) {
                continue;
            }
            batch.box(
                hitbox.position, hitbox.getHalfSize() * 2.0f, glm::vec4(1.0f)
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
        auto view = registry->view<Transform, rigging::Skeleton>();
        auto ctx = pctx.sub(&batch);
        ctx.setDepthTest(false);
        ctx.setDepthMask(false);
        ctx.setLineWidth(2);
        for (auto [entity, transform, skeleton] : view.each()) {
            auto config = skeleton.config;
            if (config == nullptr) {
                continue;
            }
            const auto& pos = transform.pos;
            const auto& size = transform.size;
            if (frustum && !frustum->isBoxVisible(pos - size, pos + size)) {
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
    auto view = registry->view<EntityId, Transform, rigging::Skeleton>();
    for (auto [entity, eid, transform, skeleton] : view.each()) {
        if (eid.uid == fpsEntity) {
            continue;
        }
        const auto& pos = transform.pos;
        const auto& size = transform.size;
        if (frustum && !frustum->isBoxVisible(pos - size, pos + size)) {
            continue;
        }

        const auto* rigConfig = skeleton.config;
        if (rigConfig) {
            rigConfig->render(
                assets, batch, skeleton, transform.rot, pos, size
            );
        }
    }
}

bool Entities::hasBlockingInside(AABB aabb) {
    auto view = registry->view<EntityId, Rigidbody>();
    for (auto [entity, eid, body] : view.each()) {
        if (eid.def.blocking && aabb.intersects(body.hitbox.getAABB(), -0.05f)) {
            return true;
        }
    }
    return false;
}

std::vector<Entity> Entities::getAllInside(AABB aabb) {
    std::vector<Entity> collected;
    auto view = registry->view<EntityId, Transform>();
    for (auto [entity, eid, transform] : view.each()) {
        if (!eid.destroyFlag && aabb.contains(transform.pos)) {
            const auto& found = uids.find(entity);
            if (found == uids.end()) {
                continue;
            }
            if (auto wrapper = get(found->second)) {
                collected.push_back(*wrapper);
            }
        }
    }
    return collected;
}

std::vector<Entity> Entities::getAllInRadius(glm::vec3 center, float radius) {
    std::vector<Entity> collected;
    auto view = registry->view<Transform>();
    for (auto [entity, transform] : view.each()) {
        if (glm::distance2(transform.pos, center) <= radius * radius) {
            const auto& found = uids.find(entity);
            if (found == uids.end()) {
                continue;
            }
            if (auto wrapper = get(found->second)) {
                collected.push_back(*wrapper);
            }
        }
    }
    return collected;
}
