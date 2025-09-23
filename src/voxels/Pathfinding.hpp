#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Block;
class Level;
class GlobalChunks;

template <typename T>
class ContentUnitIndices;

namespace voxels {
    struct RouteNode {
        glm::ivec3 pos;
    };

    struct Route {
        bool found;
        std::vector<RouteNode> nodes;
        int totalVisited;
    };

    struct Node {
        glm::ivec3 pos;
        glm::ivec3 parent;
        float gScore;
        float fScore;
    };

    struct NodeLess {
        bool operator()(const Node& l, const Node& r) const {
            return l.fScore > r.fScore;
        }
    };

    struct State {
        std::priority_queue<Node, std::vector<Node>, NodeLess> queue;
        std::unordered_set<glm::ivec3> blocked;
        std::unordered_map<glm::ivec3, Node> parents;
        glm::ivec3 nearest;
        float minHScore;
        bool finished = true;
    };

    struct Agent {
        bool enabled = false;
        bool mayBeIncomplete = true;
        int height = 2;
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
        const ContentUnitIndices<Block>& blockDefs;
        std::unordered_map<int, Agent> agents;
        int nextAgent = 1;

        int getSurfaceAt(
            const Agent& agent, const glm::ivec3& pos, int maxDelta, float& cost
        );

        int checkPoint(const Agent& agent, int x, int y, int z, int& cost);
    };
}
