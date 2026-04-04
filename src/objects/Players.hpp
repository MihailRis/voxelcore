#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>

#include "typedefs.hpp"
#include "interfaces/Serializable.hpp"

inline constexpr float DEF_PLAYER_Y = 100.0f;
inline constexpr float DEF_PLAYER_SPEED = 4.0f;
inline constexpr int DEF_PLAYER_INVENTORY_SIZE = 40;

class Level;
class Player;

class Players : public Serializable {
public:
    static inline int64_t NONE = -1;
private:
    Level& level;
    std::unordered_map<int64_t, std::unique_ptr<Player>> players;

    void add(std::unique_ptr<Player> player);
public:
    Players(Level& level);

    Player* get(int64_t id) const;

    Player* create(int64_t id=NONE);

    void suspend(int64_t id);

    void resume(int64_t id);

    void remove(int64_t id);

    std::vector<Player*> getAllInRadius(const glm::vec3& center, float radius) const;
    std::vector<Player*> getAll() const;
    Player* getNearest(const glm::vec3& position) const;

    dv::value serialize() const override;

    void deserialize(const dv::value& src) override;

    auto begin() const {
        return players.begin();
    }

    auto end() const {
        return players.end();
    }

    size_t size() const {
        return players.size();
    }
};
