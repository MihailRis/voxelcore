#include "Pathfinding.hpp"

#include "content/Content.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/blocks_agent.hpp"
#include "world/Level.hpp"

inline constexpr float SQRT2 = 1.4142135623730951f;  // sqrt(2)

using namespace voxels;

static float heuristic(const glm::ivec3& a, const glm::ivec3& b) {
    return glm::distance(glm::vec3(a), glm::vec3(b));
}

Pathfinding::Pathfinding(const Level& level)
    : level(level),
      chunks(*level.chunks),
      blockDefs(level.content.getIndices()->blocks) {
}

static bool check_passability(
    const Agent& agent,
    const GlobalChunks& chunks,
    const Node& node,
    const glm::ivec2& offset,
    bool diagonal
) {
    if (!diagonal) {
        return true;
    }
    auto a = node.pos + glm::ivec3(offset.x, 0, 0);
    auto b = node.pos + glm::ivec3(0, 0, offset.y);

    for (int i = 0; i < agent.height; i++) {
        if (blocks_agent::is_obstacle_at(chunks, a.x, a.y + i, a.z))
            return false;
        if (blocks_agent::is_obstacle_at(chunks, b.x, b.y + i, b.z))
            return false;
    }
    return true;
}

static void restore_route(
    Route& route,
    const glm::ivec3& lastPos,
    const std::unordered_map<glm::ivec3, Node>& parents
) {
    auto pos = lastPos;
    while (true) {
        const auto& found = parents.find(pos);
        if (found == parents.end()) {
            route.nodes.push_back({pos});
            break;
        }
        route.nodes.push_back({pos});
        pos = found->second.pos;
    }
}

int Pathfinding::createAgent() {
    int id = nextAgent++;
    agents[id] = Agent();
    return id;
}

bool Pathfinding::removeAgent(int id) {
    auto found = agents.find(id);
    if (found != agents.end()) {
        agents.erase(found);
        return true;
    }
    return false;
}

void Pathfinding::performAllAsync(int stepsPerAgent) {
    for (auto& [_, agent] : agents) {
        if (agent.state.finished) {
            continue;
        }
        perform(agent, stepsPerAgent);
    }
}

static Route finish_route(Agent& agent, State&& state) {
    Route route {};
    restore_route(route, state.nearest, state.parents);
    route.totalVisited = state.blocked.size();
    route.nodes.push_back({agent.start});
    route.found = true;
    state.finished = true;
    agent.state = std::move(state);
    agent.route = route;
    return route;
}

enum Passability {
    NON_PASSABLE = -1,
    OBSTACLE = 0,
    PASSABLE = 1,
};

Route Pathfinding::perform(Agent& agent, int maxVisited) {
    using namespace blocks_agent;

    State state = std::move(agent.state);
    if (state.queue.empty()) {
        state.queue.push(
            {agent.start, {}, 0, heuristic(agent.start, agent.target)}
        );
    }

    const auto& chunks = *level.chunks;
    int height = std::max(agent.height, 1);

    if (state.nearest == glm::ivec3(0)) {
        state.nearest = agent.start;
        state.minHScore = heuristic(agent.start, agent.target);
    }
    int visited = -1;

    while (!state.queue.empty()) {
        if (state.blocked.size() == agent.maxVisitedBlocks) {
            if (agent.mayBeIncomplete) {
                return finish_route(agent, std::move(state));
            }
            break;
        }
        visited++;
        if (visited == maxVisited) {
            state.finished = false;
            agent.state = std::move(state);
            return {};
        }

        auto node = state.queue.top();
        state.queue.pop();

        if (node.pos.x == agent.target.x &&
            glm::abs((node.pos.y - agent.target.y) / height) == 0 &&
            node.pos.z == agent.target.z) {
            return finish_route(agent, std::move(state));
        }

        state.blocked.emplace(node.pos);
        glm::ivec2 neighbors[8] {
            {0, 1},
            {1, 0},
            {0, -1},
            {-1, 0},
            {-1, -1},
            {1, -1},
            {1, 1},
            {-1, 1},
        };

        for (int i = 0; i < sizeof(neighbors) / sizeof(glm::ivec2); i++) {
            auto offset = neighbors[i];
            auto pos = node.pos;

            float cost = glm::abs(node.pos.y - pos.y) * 10;
            int surface = getSurfaceAt(
                agent, pos + glm::ivec3(offset.x, 0, offset.y), 1, cost
            );

            if (surface == NON_PASSABLE) {
                continue;
            }
            pos.y = surface;
            auto point = pos + glm::ivec3(offset.x, 0, offset.y);
            if (state.blocked.find(point) != state.blocked.end()) {
                continue;
            }

            if (is_obstacle_at(
                    chunks, pos.x, pos.y + agent.jumpHeight, pos.z
                )) {
                continue;
            }
            if (!check_passability(agent, chunks, node, offset, i >= 4)) {
                continue;
            }

            float sum = glm::abs(offset.x) + glm::abs(offset.y);
            float gScore = node.gScore + sum + cost;
            const auto& found = state.parents.find(point);
            if (found == state.parents.end()) {
                float hScore = heuristic(point, agent.target);
                if (hScore < state.minHScore) {
                    state.minHScore = hScore;
                    state.nearest = point;
                }
                float fScore = gScore * 0.75f + hScore;
                Node nNode {point, node.pos, gScore, fScore};
                state.parents[point] = node;
                state.queue.push(nNode);
            }
        }
    }
    state.finished = true;
    agent.state = std::move(state);
    return finish_route(agent, std::move(agent.state));
}

Agent* Pathfinding::getAgent(int id) {
    const auto& found = agents.find(id);
    if (found != agents.end()) {
        return &found->second;
    }
    return nullptr;
}

const std::unordered_map<int, Agent>& Pathfinding::getAgents() const {
    return agents;
}

int Pathfinding::checkPoint(const Agent& agent, int x, int y, int z, int& cost) {
    auto vox = blocks_agent::get(chunks, x, y, z);
    if (vox == nullptr) {
        return OBSTACLE;
    }
    const auto& def = blockDefs.require(vox->id);
    if (def.obstacle) {
        return OBSTACLE;
    }
    for (const auto& pair : agent.avoidTags) {
        if (def.rt.tags.find(pair.first) != def.rt.tags.end()) {
            cost = pair.second;
            return NON_PASSABLE;
        }
    }
    return PASSABLE;
}

int Pathfinding::getSurfaceAt(
    const Agent& agent, const glm::ivec3& pos, int maxDelta, float& cost
) {
    using namespace blocks_agent;

    int status;
    int surface = pos.y;
    int ncost = 0;
    if ((status = checkPoint(agent, pos.x, surface, pos.z, ncost)) == OBSTACLE) {
        if ((status = checkPoint(agent, pos.x, surface + 1, pos.z, ncost)) == OBSTACLE) {
            return NON_PASSABLE;
        } else if (status == NON_PASSABLE) {
            cost += 5;
        }
        cost += ncost;
        return surface + 1;
    } else {
        if (status == NON_PASSABLE) {
            cost += 5;
        }
        if ((status = checkPoint(agent, pos.x, surface - 1, pos.z, ncost)) == OBSTACLE) {
            cost += ncost;
            return surface;
        } else if (status == NON_PASSABLE) {
            cost += 5;
        }
        if ((status = checkPoint(agent, pos.x, surface - 2, pos.z, ncost)) == OBSTACLE) {
            cost += ncost;
            return surface - 1;
        }
        return NON_PASSABLE;
    }
    return NON_PASSABLE;
}
