#include "Players.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include "Player.hpp"
#include "items/Inventories.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "objects/Entities.hpp"

Players::Players(Level& level) : level(level) {}

void Players::add(std::unique_ptr<Player> player) {
    players[player->getId()] = std::move(player);
}

Player* Players::get(int64_t id) const {
    const auto& found = players.find(id);
    if (found == players.end()) {
        return nullptr;
    }
    return found->second.get();
}

std::vector<Player*> Players::getAllInRadius(
    const glm::vec3& center, float radius
) const {
    std::vector<Player*> foundPlayers;

    for (const auto& pair : players) {
        auto player = pair.second.get();
        auto relativePos = player->getPosition() - center;
        if (!player->isSuspended() && glm::length2(relativePos) <= radius) {
            foundPlayers.emplace_back(player);
        }
    }
    return foundPlayers;
}

std::vector<Player*> Players::getAll() const {
    std::vector<Player*> allPlayers;
    allPlayers.reserve(players.size());
    for (const auto& pair : players) {
        allPlayers.emplace_back(pair.second.get());
    }
    return allPlayers;
}

Player* Players::getNearest(const glm::vec3& position) const {
    Player* nearest = nullptr;
    float nearestDist2 = std::numeric_limits<float>::max();
    for (const auto& pair : players) {
        auto player = pair.second.get();
        if (player->isSuspended()) {
            continue;
        }
        auto relativePos = player->getPosition() - position;
        auto dist2 = glm::length2(relativePos);
        if (dist2 < nearestDist2) {
            nearestDist2 = dist2;
            nearest = player;
        }
    }
    return nearest;
}

Player* Players::create(int64_t id) {
    int64_t& nextPlayerID = level.getWorld()->getInfo().nextPlayerId;
    if (id == NONE) {
        id = nextPlayerID++;
    } else {
        if (auto player = get(id)) {
            return player;
        }
        nextPlayerID = std::max(id + 1, nextPlayerID);
    }
    auto playerPtr = std::make_unique<Player>(
        level,
        id,
        "",
        glm::vec3(0, DEF_PLAYER_Y, 0),
        DEF_PLAYER_SPEED,
        level.inventories->create(DEF_PLAYER_INVENTORY_SIZE),
        ENTITY_AUTO
    );
    auto player = playerPtr.get();
    add(std::move(playerPtr));

    level.inventories->store(player->getInventory());
    return player;
}

void Players::suspend(int64_t id) {
    if (auto player = get(id)) {
        if (player->isSuspended()) {
            return;
        }
        player->setSuspended(true);
        level.entities->despawn(player->getEntity());
        player->setEntity(0);
    }
}

void Players::resume(int64_t id) {
    if (auto player = get(id)) {
        if (!player->isSuspended()) {
            return;
        }
        player->setSuspended(false);
    }
}

void Players::remove(int64_t id) {
    players.erase(id);
}

dv::value Players::serialize() const {
    auto root = dv::object();
    auto& list = root.list("players");

    for (const auto& [id, player] : players) {
        list.add(player->serialize());
    }
    return root;
}

void Players::deserialize(const dv::value& src) {
    players.clear();

    const auto& players = src["players"];
    for (auto& playerMap : players) {
        auto playerPtr = std::make_unique<Player>(
            level,
            0,
            "",
            glm::vec3(0, DEF_PLAYER_Y, 0),
            DEF_PLAYER_SPEED,
            level.inventories->create(DEF_PLAYER_INVENTORY_SIZE),
            ENTITY_AUTO
        );
        auto player = playerPtr.get();
        player->deserialize(playerMap);
        add(std::move(playerPtr));
        auto& inventory = player->getInventory();
        // invalid inventory id pre 0.25
        if (inventory->getId() == 0) {
            inventory->setId(level.getWorld()->getNextInventoryId());
        }
        level.inventories->store(player->getInventory());
    }
}
