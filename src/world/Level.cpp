#include "Level.hpp"

#include "content/Content.hpp"
#include "data/dv_util.hpp"
#include "items/Inventories.hpp"
#include "items/Inventory.hpp"
#include "objects/Entities.hpp"
#include "objects/Entity.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "physics/Hitbox.hpp"
#include "physics/PhysicsSolver.hpp"
#include "settings.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/Pathfinding.hpp"
#include "window/Camera.hpp"
#include "LevelEvents.hpp"
#include "World.hpp"

Level::Level(
    std::unique_ptr<World> worldPtr,
    const Content& content,
    EngineSettings& settings
)
    : world(std::move(worldPtr)),
      content(content),
      chunks(std::make_unique<GlobalChunks>(*this)),
      physics(std::make_unique<PhysicsSolver>(glm::vec3(0, -22.6f, 0))),
      events(std::make_unique<LevelEvents>()),
      entities(std::make_unique<Entities>(*this)),
      players(std::make_unique<Players>(*this)),
      pathfinding(std::make_unique<voxels::Pathfinding>(*this)) {
    const auto& worldInfo = world->getInfo();
    auto& cameraIndices = content.getIndices(ResourceType::CAMERA);
    for (size_t i = 0; i < cameraIndices.size(); i++) {
        auto camera = std::make_shared<Camera>();
        auto map = cameraIndices.getSavedData(i);
        if (map != nullptr) {
            dv::get_vec(map, "pos", camera->position);
            dv::get_mat(map, "rot", camera->rotation);
            map.at("perspective").get(camera->perspective);
            map.at("flipped").get(camera->flipped);
            map.at("zoom").get(camera->zoom);
            float fov = camera->getFov();
            map.at("fov").get(fov);
            camera->setFov(fov);
        }
        camera->updateVectors();
        cameras.push_back(std::move(camera));
    }

    if (worldInfo.nextEntityId) {
        entities->setNextID(worldInfo.nextEntityId);
    }

    events->listen(LevelEventType::CHUNK_SHOWN, [this](LevelEventType, Chunk* chunk) {
        chunks->incref(chunk);
    });
    events->listen(LevelEventType::CHUNK_HIDDEN, [this](LevelEventType, Chunk* chunk) {
        chunks->decref(chunk);
    });
    chunks->setOnUnload([this](Chunk& chunk) {
        events->trigger(LevelEventType::CHUNK_UNLOAD, &chunk);
        AABB aabb = chunk.getAABB();
        entities->despawn(entities->getAllInside(aabb));
    });
    inventories = std::make_unique<Inventories>(*this);
}

Level::~Level() = default;

World* Level::getWorld() {
    return world.get();
}

const World* Level::getWorld() const {
    return world.get();
}

void Level::onSave() {
    auto& cameraIndices = content.getIndices(ResourceType::CAMERA);
    for (size_t i = 0; i < cameraIndices.size(); i++) {
        auto& camera = *cameras.at(i);
        auto map = dv::object();
        map["pos"] = dv::to_value(camera.position);
        map["rot"] = dv::to_value(camera.rotation);
        map["perspective"] = camera.perspective;
        map["flipped"] = camera.flipped;
        map["zoom"] = camera.zoom;
        map["fov"] = camera.getFov();
        cameraIndices.saveData(i, std::move(map));
    }
}

std::shared_ptr<Camera> Level::getCamera(const std::string& name) {
    size_t index = content.getIndices(ResourceType::CAMERA).indexOf(name);
    if (index == ResourceIndices::MISSING) {
        return nullptr;
    }
    return cameras.at(index);
}
