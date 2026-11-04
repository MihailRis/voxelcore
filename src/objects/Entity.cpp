#include "Entity.hpp"

#include "animation/rigging.hpp"
#include "Entities.hpp"
#include "EntityDef.hpp"
#include "logic/scripting/scripting.hpp"
#include "Rigidbody.hpp"
#include "ScriptComponents.hpp"
#include "Transform.hpp"

static inline std::string SAVED_DATA_VARNAME = "SAVED_DATA";

Entity::Entity(
    Entities& entities,
    EntityId id,
    Transform transform,
    Rigidbody rigidbody,
    std::unique_ptr<rigging::Skeleton> skeleton
)
    : entities(entities),
      id(std::move(id)),
      transform(std::move(transform)),
      rigidbody(std::move(rigidbody)),
      skeleton(std::move(skeleton)) {
}

Entity::~Entity() = default;

void Entity::setInterpolatedPosition(const glm::vec3& position) {
    if (skeleton) {
        skeleton->interpolation.refresh(position);
    }
}

glm::vec3 Entity::getInterpolatedPosition() const {
    if (skeleton) {
        const auto& interpolation = skeleton->interpolation;
        if (interpolation.isEnabled() && interpolation.isRefreshed()) {
            return interpolation.getCurrent();
        }
    }
    return transform.pos;
}

void Entity::setInterpolatedRotation(const glm::quat& rotation) {
    if (skeleton) {
        skeleton->rotationInterpolation.refresh(rotation);
    }
}

void Entity::destroy() {
    entities.despawn(id.uid);
}

void Entity::setRig(std::shared_ptr<const rigging::SkeletonConfig> rigConfig) {
    if (skeleton) {
        skeleton->setConfig(std::move(rigConfig));
    }
}

dv::value Entity::serialize() const {
    const auto& def = id.def;

    auto root = dv::object();
    root["def"] = def.name;
    root["uid"] = id.uid;

    root[COMP_TRANSFORM] = transform.serialize();
    root[COMP_RIGIDBODY] =
        rigidbody.serialize(def.save.body.velocity, def.save.body.settings);

    if (skeleton != nullptr && skeleton->config != nullptr) {
        if (skeleton->config->getName() != def.skeletonName) {
            root["skeleton-name"] = skeleton->config->getName();
        }
        if (def.save.skeleton.pose || def.save.skeleton.textures) {
            root[COMP_SKELETON] = skeleton->serialize(
                def.save.skeleton.pose, def.save.skeleton.textures
            );
        }
    }
    if (!scripting.components.empty()) {
        auto& compsMap = root.object("comps");
        for (auto& comp : scripting.components) {
            if (comp->env == nullptr) {
                continue;
            }
            auto data =
                scripting::get_component_value(comp->env, SAVED_DATA_VARNAME);
            compsMap[comp->name] = data;
        }
    }
    return root;
}

void Entity::setPlayer(int64_t player) {
    if (id.player == player) {
        return;
    }
    id.player = player;
    scripting::on_entity_player_set(*this, player);
}

