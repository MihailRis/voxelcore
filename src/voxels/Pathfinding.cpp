#include "Pathfinding.hpp"

#include "constants.hpp"
#include "content/Content.hpp"
#include "maths/voxmaths.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/GlobalChunks.hpp"
#include "world/Level.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

static constexpr float SQRT2 = 1.4142135623730951f;  // sqrt(2)
static constexpr int16_t SOLID = -1;
static constexpr int16_t UNKNOWN = std::numeric_limits<int16_t>::min();
static constexpr int NO_SURFACE = std::numeric_limits<int>::min();
static constexpr int MAX_DROP = 1;
static constexpr float SEGMENT_SAMPLE_STEP = 0.2f;
static constexpr size_t MAX_SEGMENT_NODES = 32;

using namespace voxels;

static const glm::ivec2 NEIGHBORS[] {
    {0, 1}, {1, 0}, {0, -1}, {-1, 0},
    {-1, -1}, {1, -1}, {1, 1}, {-1, 1},
};

static float heuristic(const glm::ivec3& a, const glm::ivec3& b) {
    return glm::distance(glm::vec3(a), glm::vec3(b));
}

class BlocksSampler {
public:
    BlocksSampler(
        const GlobalChunks& chunks,
        const ContentUnitIndices<Block, blockid_t>& defs,
        const Agent& agent,
        std::vector<int16_t>& costs
    )
        : height(std::max(agent.height, 1)),
          radius(static_cast<int>(
              std::ceil(std::max(agent.width - 1.0f, 0.0f) * 0.5f - 1e-4f)
          )),
          halfWidth(std::max(agent.width * 0.5f, 0.05f)),
          jumpHeight(std::max(agent.jumpHeight, 0)),
          chunks(chunks),
          defs(defs),
          agent(agent),
          costs(costs) {
        if (costs.size() != defs.count()) {
            costs.assign(defs.count(), UNKNOWN);
        }
    }

    int16_t at(int x, int y, int z) {
        if (y < 0) {
            return SOLID;
        }
        if (y >= CHUNK_H) {
            return 0;
        }
        int cx = floordiv<CHUNK_W>(x);
        int cz = floordiv<CHUNK_D>(z);
        if (!hasChunk || cx != chunkX || cz != chunkZ) {
            chunk = chunks.getChunk(cx, cz);
            chunkX = cx;
            chunkZ = cz;
            hasChunk = true;
        }
        if (chunk == nullptr) {
            return SOLID;
        }
        int lx = x - cx * CHUNK_W;
        int lz = z - cz * CHUNK_D;
        blockid_t id = chunk->voxels[(y * CHUNK_D + lz) * CHUNK_W + lx].id;
        if (id >= costs.size()) {
            return SOLID;
        }
        auto& cost = costs[id];
        if (cost == UNKNOWN) {
            cost = classify(id);
        }
        return cost;
    }

    int16_t getMaxCostInBox(int x0, int z0, int x1, int z1, int y0, int y1) {
        int16_t result = 0;
        for (int y = y0; y < y1; y++) {
            for (int z = z0; z <= z1; z++) {
                for (int x = x0; x <= x1; x++) {
                    int16_t cost = at(x, y, z);
                    if (cost == SOLID) {
                        return SOLID;
                    }
                    result = std::max(result, cost);
                }
            }
        }
        return result;
    }

    int16_t getFootprintCost(int x, int z, int y0, int y1) {
        return getMaxCostInBox(
            x - radius, z - radius, x + radius, z + radius, y0, y1
        );
    }

    int16_t getStandingCost(const glm::ivec3& pos) {
        return getFootprintCost(pos.x, pos.z, pos.y, pos.y + height);
    }

    int findSurfaceHeight(int x, int y, int z) {
        if (at(x, y, z) == SOLID) {
            for (int i = 1; i <= jumpHeight; i++) {
                if (at(x, y + i, z) != SOLID) {
                    return y + i;
                }
            }
            return NO_SURFACE;
        }
        for (int i = 0; i <= MAX_DROP; i++) {
            if (at(x, y - i - 1, z) == SOLID) {
                return y - i;
            }
        }
        return NO_SURFACE;
    }

    int16_t getMovementCost(const glm::ivec3& from, const glm::ivec3& to) {
        int top = std::max(from.y, to.y) + height;
        int16_t cost = getFootprintCost(to.x, to.z, to.y, top);
        if (cost == SOLID) {
            return SOLID;
        }
        if (to.y > from.y &&
            getFootprintCost(from.x, from.z, from.y + height, top) == SOLID) {
            return SOLID;
        }
        int dx = to.x - from.x;
        int dz = to.z - from.z;
        if (dx == 0 || dz == 0) {
            return cost;
        }
        int bottom = std::min(from.y, to.y);
        int ax = from.x + dx * (radius + 1);
        int az = from.z - dz * radius;
        int bx = from.x - dx * radius;
        int bz = from.z + dz * (radius + 1);
        int16_t a = getMaxCostInBox(ax, az, ax, az, bottom, top);
        int16_t b = getMaxCostInBox(bx, bz, bx, bz, bottom, top);
        if (a == SOLID || b == SOLID) {
            return SOLID;
        }
        return std::max({cost, a, b});
    }

    int16_t getLineCost(glm::vec2 a, glm::vec2 b, int y0, int y1) {
        auto min = [this](float a, float b) {
            return static_cast<int>(std::floor(std::min(a, b) - halfWidth));
        };
        auto max = [this](float a, float b) {
            return static_cast<int>(std::ceil(std::max(a, b) + halfWidth));
        };
        int x0 = min(a.x, b.x);
        int z0 = min(a.y, b.y);
        int x1 = max(a.x, b.x);
        int z1 = max(a.y, b.y);
        return getMaxCostInBox(
            x0, z0, std::max(x0, x1 - 1), std::max(z0, z1 - 1), y0, y1
        );
    }

    bool canWalkDirectly(
        const glm::ivec3& a, const glm::ivec3& b, int16_t maxCost
    ) {
        if (b.y > a.y) {
            return false;
        }
        glm::vec2 from(a.x + 0.5f, a.z + 0.5f);
        glm::vec2 to(b.x + 0.5f, b.z + 0.5f);
        int steps = std::max(
            1,
            static_cast<int>(
                std::ceil(glm::distance(from, to) / SEGMENT_SAMPLE_STEP)
            )
        );
        int cx = a.x;
        int cz = a.z;
        int y = a.y;
        glm::vec2 prev = from;
        auto isBlocked = [&](glm::vec2 p, int y0, int y1) {
            int16_t cost = getLineCost(prev, p, y0, y1);
            return cost == SOLID || cost > maxCost;
        };
        for (int i = 1; i <= steps; i++) {
            float t = i / static_cast<float>(steps);
            glm::vec2 p = from + (to - from) * t;
            int px = static_cast<int>(std::floor(p.x));
            int pz = static_cast<int>(std::floor(p.y));
            int top = y + height;
            if (px != cx || pz != cz) {
                int ny = findSurfaceHeight(px, y, pz);
                if (ny == NO_SURFACE || ny > y) {
                    return false;
                }
                if (ny < y) {
                    if (isBlocked(p, y, top)) {
                        return false;
                    }
                    prev = p;
                }
                cx = px;
                cz = pz;
                y = ny;
            }
            if (isBlocked(p, y, top)) {
                return false;
            }
            prev = p;
        }
        return y == b.y;
    }

    std::vector<glm::ivec3> simplifyPath(
        const std::vector<glm::ivec3>& path
    ) {
        if (path.size() <= 2) {
            return path;
        }
        std::vector<int16_t> nodeCosts(path.size());
        for (size_t i = 0; i < path.size(); i++) {
            nodeCosts[i] = std::max<int16_t>(getStandingCost(path[i]), 0);
        }
        auto isDirect = [&](size_t from, size_t to) {
            int16_t maxCost = std::max(nodeCosts[from], nodeCosts[to]);
            return canWalkDirectly(path[from], path[to], maxCost);
        };
        const size_t last = path.size() - 1;

        std::vector<glm::ivec3> result {path.front()};
        size_t anchor = 0;
        while (anchor < last) {
            size_t limit = std::min(last, anchor + MAX_SEGMENT_NODES);
            size_t good = anchor + 1;
            size_t bad = limit + 1;
            for (size_t step = 2; good < limit; step *= 2) {
                size_t next = std::min(anchor + step, limit);
                if (!isDirect(anchor, next)) {
                    bad = next;
                    break;
                }
                good = next;
            }
            while (bad - good > 1) {
                size_t mid = (good + bad) / 2;
                (isDirect(anchor, mid) ? good : bad) = mid;
            }
            result.push_back(path[good]);
            anchor = good;
        }
        return result;
    }

    const int height;
    const int radius;
    const float halfWidth;
    const int jumpHeight;
private:
    const GlobalChunks& chunks;
    const ContentUnitIndices<Block, blockid_t>& defs;
    const Agent& agent;
    std::vector<int16_t>& costs;
    const Chunk* chunk = nullptr;
    int chunkX = 0;
    int chunkZ = 0;
    bool hasChunk = false;

    int16_t classify(blockid_t id) const {
        const auto& def = defs.require(id);
        if (def.obstacle) {
            return SOLID;
        }
        int cost = 0;
        for (const auto& [tag, tagCost] : agent.avoidTags) {
            if (def.rt.tags.find(tag) != def.rt.tags.end()) {
                cost = std::max(cost, tagCost);
            }
        }
        return static_cast<int16_t>(
            std::clamp(cost, 0, static_cast<int>(INT16_MAX))
        );
    }
};

Pathfinding::Pathfinding(const Level& level)
    : level(level),
      chunks(*level.chunks),
      blockDefs(level.content.getIndices()->blocks) {
}

static Route finish_route(
    Agent& agent, BlocksSampler& sampler, const glm::ivec3& last, bool found
) {
    auto& state = agent.state;
    std::vector<glm::ivec3> path;
    for (auto pos = last;;) {
        path.push_back(pos);
        const auto& visit = state.visits.find(pos);
        if (visit == state.visits.end() || visit->second.parent == pos) {
            break;
        }
        pos = visit->second.parent;
    }
    std::reverse(path.begin(), path.end());
    path = sampler.simplifyPath(path);

    Route route {};
    route.found = found;
    route.totalVisited = state.closedCount;
    route.nodes.reserve(path.size());

    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        route.nodes.push_back({*it});
    }

    state.finished = true;
    state.queue = {};
    state.visits = {};
    agent.route = route;
    return route;
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

Route Pathfinding::perform(Agent& agent, int maxVisited) {
    auto& state = agent.state;
    BlocksSampler sampler(chunks, blockDefs, agent, state.blockCosts);

    if (state.visits.empty()) {
        float hScore = heuristic(agent.start, agent.target);
        state.queue.push({agent.start, 0.0f, hScore});
        state.visits[agent.start] = {agent.start, 0.0f, false};
        state.closedCount = 0;
        state.nearest = agent.start;
        state.minHScore = hScore;
    }

    int visited = 0;
    while (!state.queue.empty()) {
        if (state.closedCount >= agent.maxVisitedBlocks) {
            return finish_route(
                agent, sampler, state.nearest, agent.mayBeIncomplete
            );
        }
        if (visited++ == maxVisited) {
            state.finished = false;
            return {};
        }

        auto node = state.queue.top();
        state.queue.pop();

        auto& current = state.visits[node.pos];
        if (current.closed) {
            continue;
        }
        current.closed = true;
        state.closedCount++;
        float gScore = current.gScore;

        if (node.pos.x == agent.target.x && node.pos.z == agent.target.z &&
            glm::abs(node.pos.y - agent.target.y) < sampler.height) {
            return finish_route(agent, sampler, node.pos, true);
        }

        for (size_t i = 0; i < std::size(NEIGHBORS); i++) {
            const auto& offset = NEIGHBORS[i];
            glm::ivec3 point(node.pos.x + offset.x, 0, node.pos.z + offset.y);
            point.y = sampler.findSurfaceHeight(point.x, node.pos.y, point.z);
            if (point.y == NO_SURFACE) {
                continue;
            }
            auto found = state.visits.find(point);
            if (found != state.visits.end() && found->second.closed) {
                continue;
            }
            int16_t cost = sampler.getMovementCost(node.pos, point);
            if (cost == SOLID) {
                continue;
            }
            float nextGScore = gScore + (i >= 4 ? SQRT2 : 1.0f) + cost;
            if (found != state.visits.end() &&
                found->second.gScore <= nextGScore) {
                continue;
            }
            float hScore = heuristic(point, agent.target);
            if (hScore < state.minHScore) {
                state.minHScore = hScore;
                state.nearest = point;
            }
            state.visits[point] = {node.pos, nextGScore, false};
            state.queue.push({point, nextGScore, nextGScore * 0.75f + hScore});
        }
    }
    return finish_route(agent, sampler, state.nearest, agent.mayBeIncomplete);
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
