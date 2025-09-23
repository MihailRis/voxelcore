#include "Player.hpp"

#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <utility>

#include "content/ContentReport.hpp"
#include "items/Inventory.hpp"
#include "Entities.hpp"
#include "Entity.hpp"
#include "rigging.hpp"
#include "physics/Hitbox.hpp"
#include "physics/PhysicsSolver.hpp"
#include "voxels/Chunks.hpp"
#include "window/Camera.hpp"
#include "world/Level.hpp"
#include "data/dv_util.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("player");

constexpr int SPAWN_ATTEMPTS_PER_UPDATE = 64;

Player::Player(
    Level& level,
    int64_t id,
    const std::string& name,
    glm::vec3 position,
    float speed,
    std::shared_ptr<Inventory> inv,
    entityid_t eid
)
    : level(level),
      id(id),
      name(name),
      speed(speed),
      chosenSlot(0),
      position(position),
      inventory(std::move(inv)),
      eid(eid),
      chunks(std::make_unique<Chunks>(
          3, 3, 0, 0, level.events.get(), *level.content.getIndices()
      )),
      fpCamera(level.getCamera("core:first-person")),
      spCamera(level.getCamera("core:third-person-front")),
      tpCamera(level.getCamera("core:third-person-back")),
      currentCamera(fpCamera) {
    fpCamera->setFov(glm::radians(90.0f));
    spCamera->setFov(glm::radians(90.0f));
    tpCamera->setFov(glm::radians(90.0f));
}

Player::~Player() = default;

void Player::updateEntity() {
    if (eid == ENTITY_AUTO) {
        const auto& defaults = level.content.getDefaults();
        const auto& defName = defaults["player-entity"].asString();
        if (!defName.empty()) {
            auto& def = level.content.entities.require(defName);
            eid = level.entities->spawn(def, getPosition());
            if (auto entity = level.entities->get(eid)) {
                entity->setPlayer(id);
            }
        }
    } else if (auto entity = level.entities->get(eid)) {
        position = entity->getTransform().pos;
        if (auto entity = level.entities->get(eid)) {
            entity->setPlayer(id);
        }
    } else if (chunks->getChunkByVoxel(position) && eid != ENTITY_NONE) {
        logger.error() << "player entity despawned or deleted; "
                          "will be respawned";
        eid = ENTITY_AUTO;
    }
}

Hitbox* Player::getHitbox() {
    if (auto entity = level.entities->get(eid)) {
        return &entity->getRigidbody().hitbox;
    }
    return nullptr;
}

void Player::updateSelectedEntity() {
    selectedEid = selection.entity;
}

void Player::postUpdate() {
    auto entity = level.entities->get(eid);
    if (!entity.has_value()) {
        return;
    }
    auto& hitbox = entity->getRigidbody().hitbox;
    position = hitbox.position;

    if (flight && hitbox.grounded && !noclip) {
        flight = false;
    }
    if (spawnpoint.y <= 0.1) {
        for (int i = 0; i < SPAWN_ATTEMPTS_PER_UPDATE; i++) {
            attemptToFindSpawnpoint();
        }
    }
}

void Player::teleport(glm::vec3 position) {
    this->position = position;

    if (auto entity = level.entities->get(eid)) {
        entity->getRigidbody().hitbox.position = position;
        entity->getTransform().setPos(position);
        entity->setInterpolatedPosition(position);
    }
}

void Player::attemptToFindSpawnpoint() {
    glm::vec3 newpos(
        position.x + (rand() % 200 - 100),
        rand() % 80 + 100,
        position.z + (rand() % 200 - 100)
    );
    while (newpos.y > 0 &&
           !chunks->isObstacleBlock(newpos.x, newpos.y - 2, newpos.z)) {
        newpos.y--;
    }

    voxel* headvox = chunks->get(newpos.x, newpos.y + 1, newpos.z);
    if (chunks->isObstacleBlock(newpos.x, newpos.y, newpos.z) ||
        headvox == nullptr || headvox->id != 0) {
        return;
    }
    spawnpoint = newpos + glm::vec3(0.5f, 0.0f, 0.5f);
    teleport(spawnpoint);
}

void Player::setChosenSlot(int index) {
    chosenSlot = index;
}

int Player::getChosenSlot() const {
    return chosenSlot;
}

float Player::getSpeed() const {
    return speed;
}

bool Player::isSuspended() const {
    return suspended;
}

void Player::setSuspended(bool flag) {
    suspended = flag;
}

bool Player::isFlight() const {
    return flight;
}

void Player::setFlight(bool flag) {
    this->flight = flag;
}

bool Player::isNoclip() const {
    return noclip;
}

void Player::setNoclip(bool flag) {
    this->noclip = flag;
}

bool Player::isInfiniteItems() const {
    return infiniteItems;
}

void Player::setInfiniteItems(bool flag) {
    infiniteItems = flag;
}

bool Player::isInstantDestruction() const {
    return instantDestruction;
}

void Player::setInstantDestruction(bool flag) {
    instantDestruction = flag;
}

bool Player::isLoadingChunks() const {
    return loadingChunks;
}

void Player::setLoadingChunks(bool flag) {
    loadingChunks = flag;
}

entityid_t Player::getEntity() const {
    return eid;
}

void Player::setEntity(entityid_t eid) {
    this->eid = eid;
}

entityid_t Player::getSelectedEntity() const {
    return selectedEid;
}

void Player::setName(const std::string& name) {
    this->name = name;
}

const std::string& Player::getName() const {
    return name;
}

const std::shared_ptr<Inventory>& Player::getInventory() const {
    return inventory;
}

void Player::setSpawnPoint(glm::vec3 spawnpoint) {
    this->spawnpoint = spawnpoint;
}

glm::vec3 Player::getSpawnPoint() const {
    return spawnpoint;
}

glm::vec3 Player::getRotation(bool interpolated) const {
    if (interpolated) {
        return rotationInterpolation.getCurrent();
    }
    return rotation;
}

void Player::setRotation(const glm::vec3& rotation) {
    this->rotation = rotation;
    rotationInterpolation.refresh(rotation);
}

dv::value Player::serialize() const {
    auto root = dv::object();

    root["id"] = id;
    root["name"] = name;

    root["position"] = dv::to_value(position);
    root["rotation"] = dv::to_value(rotation);
    root["spawnpoint"] = dv::to_value(spawnpoint);

    root["flight"] = flight;
    root["noclip"] = noclip;
    root["suspended"] = suspended;
    root["infinite-items"] = infiniteItems;
    root["instant-destruction"] = instantDestruction;
    root["loading-chunks"] = loadingChunks;
    root["chosen-slot"] = chosenSlot;
    root["entity"] = eid;
    root["inventory"] = inventory->serialize();
    auto found =
        std::find(level.cameras.begin(), level.cameras.end(), currentCamera);
    if (found != level.cameras.end()) {
        root["camera"] = level.content.getIndices(ResourceType::CAMERA)
                .getName(found - level.cameras.begin());
    }
    return root;
}

void Player::deserialize(const dv::value& src) {
    src.at("id").get(id);
    src.at("name").get(name);

    const auto& posarr = src["position"];

    dv::get_vec(posarr, position);
    fpCamera->position = position;

    const auto& rotarr = src["rotation"];
    dv::get_vec(rotarr, rotation);

    const auto& sparr = src["spawnpoint"];
    setSpawnPoint(glm::vec3(
        sparr[0].asNumber(), sparr[1].asNumber(), sparr[2].asNumber()));

    flight = src["flight"].asBoolean();
    noclip = src["noclip"].asBoolean();
    src.at("suspended").get(suspended);
    src.at("infinite-items").get(infiniteItems);
    src.at("instant-destruction").get(instantDestruction);
    src.at("loading-chunks").get(loadingChunks);

    setChosenSlot(src["chosen-slot"].asInteger());
    eid = src["entity"].asNumber();

    if (src.has("inventory")) {
        getInventory()->deserialize(src["inventory"]);
    }

    if (src.has("camera")) {
        std::string name = src["camera"].asString();
        if (auto camera = level.getCamera(name)) {
            currentCamera = camera;
        }
    }
}

void Player::convert(dv::value& data, const ContentReport* report) {
    if (data.has("players")) {
        auto& players = data["players"];
        for (uint i = 0; i < players.size(); i++) {
            auto& playerData = players[i];
            if (playerData.has("inventory")) {
                Inventory::convert(playerData["inventory"], report);
            }
        }

    } else if (data.has("inventory")){
        Inventory::convert(data["inventory"], report);
    }
}
