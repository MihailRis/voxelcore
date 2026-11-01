#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <cstdint>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

#include "typedefs.hpp"

class Block;
class Level;
class GlobalChunks;

template <typename T, typename IdType>
class ContentUnitIndices;

namespace voxels {
    struct RouteNode {
        glm::ivec3 pos;
    };

    struct Route {
        std::vector<RouteNode> nodes;
        int totalVisited;
        bool found;
    };

    struct Node {
        glm::ivec3 pos;
        float gScore;
        float fScore;
    };

    struct NodeLess {
        bool operator()(const Node& l, const Node& r) const {
            return l.fScore > r.fScore;
        }
    };

    struct Visit {
        glm::ivec3 parent;
        float gScore;
        bool closed;
    };

    struct State {
        std::priority_queue<Node, std::vector<Node>, NodeLess> queue;
        std::unordered_map<glm::ivec3, Visit> visits;
        std::vector<int16_t> blockCosts;
        int closedCount = 0;
        glm::ivec3 nearest {};
        float minHScore = 0.0f;
        bool finished = true;
    };

    struct Agent {
        bool enabled = false;
        bool mayBeIncomplete = true;
        int height = 2;
        float width = 0.6f;
        int jumpHeight = 1;
        int maxVisitedBlocks = 1e3;
        glm::ivec3 start;
        glm::ivec3 target;
        Route route;
        State state {};
        std::set<std::pair<int, int>> avoidTags;
    };

    class Pathfinding {
    public:
        Pathfinding(const Level& level);

        int createAgent();

        bool removeAgent(int id);

        void performAllAsync(int stepsPerAgent);

        Route perform(Agent& agent, int maxVisited = -1);

        Agent* getAgent(int id);

        const std::unordered_map<int, Agent>& getAgents() const;
    private:
        const Level& level;
        const GlobalChunks& chunks;
        const ContentUnitIndices<Block, blockid_t>& blockDefs;
        std::unordered_map<int, Agent> agents;
        int nextAgent = 1;
    };
}
