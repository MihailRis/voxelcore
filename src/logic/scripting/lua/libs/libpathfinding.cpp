#include "api_lua.hpp"

#include "content/Content.hpp"
#include "voxels/Pathfinding.hpp"
#include "world/Level.hpp"

using namespace scripting;

static voxels::Agent* get_agent(lua::State* L) {
    return level->pathfinding->getAgent(lua::tointeger(L, 1));
}

static int l_create_agent(lua::State* L) {
    return lua::pushinteger(L, level->pathfinding->createAgent());
}

static int l_remove_agent(lua::State* L) {
    int id = lua::tointeger(L, 1);
    return lua::pushboolean(L, level->pathfinding->removeAgent(id));
}

static int l_set_enabled(lua::State* L) {
    if (auto agent = get_agent(L)) {
        agent->enabled = lua::toboolean(L, 2);
    }
    return 0;
}

static int l_is_enabled(lua::State* L) {
    if (auto agent = get_agent(L)) {
        return lua::pushboolean(L, agent->enabled);
    }
    return lua::pushboolean(L, false);
}

static int push_route(lua::State* L, const voxels::Route& route) {
    lua::createtable(L, route.nodes.size(), 1);
    for (int i = 0; i < route.nodes.size(); i++) {
        lua::pushvec3(L, route.nodes[i].pos);
        lua::rawseti(L, i + 1);
    }
    lua::pushinteger(L, route.totalVisited);
    lua::setfield(L, "total_visited");
    return 1;
}

static int l_make_route(lua::State* L) {
    if (auto agent = get_agent(L)) {
        auto start = lua::tovec3(L, 2);
        auto target = lua::tovec3(L, 3);
        agent->state = {};
        agent->start = glm::floor(start);
        agent->target = target;
        auto route = level->pathfinding->perform(*agent);
        if (!route.found) {
            return 0;
        }
        return push_route(L, route);
    }
    return 0;
}

static int l_make_route_async(lua::State* L) {
    if (auto agent = get_agent(L)) {
        auto start = lua::tovec3(L, 2);
        auto target = lua::tovec3(L, 3);
        agent->state = {};
        agent->start = glm::floor(start);
        agent->target = target;
        level->pathfinding->perform(*agent, 0);
    }
    return 0;
}

static int l_pull_route(lua::State* L) {
    if (auto agent = get_agent(L)) {
        auto& route = agent->route;
        if (!agent->state.finished) {
            return 0;
        }
        if (!route.found && !agent->mayBeIncomplete) {
            return lua::createtable(L, 0, 0);
        }
        return push_route(L, route);
    }
    return 0;
}

static int l_set_max_visited_blocks(lua::State* L) {
    if (auto agent = get_agent(L)) {
        agent->maxVisitedBlocks = lua::tointeger(L, 2);
    }
    return 0;
}

static int l_set_jump_height(lua::State* L) {
    if (auto agent = get_agent(L)) {
        agent->jumpHeight = lua::tointeger(L, 2);
    }
    return 0;
}

static int l_avoid_tag(lua::State* L) {
    if (auto agent = get_agent(L)) {
        int index =
            content->getTagIndex(std::string(lua::require_lstring(L, 2)));
        if (index != -1) {
            int cost = lua::tonumber(L, 3);
            if (cost == 0) {
                cost = 10;
            }
            agent->avoidTags.insert({index, cost});
        }
    }
    return 0;
}

const luaL_Reg pathfindinglib[] = {
    {"create_agent", lua::wrap<l_create_agent>},
    {"remove_agent", lua::wrap<l_remove_agent>},
    {"set_enabled", lua::wrap<l_set_enabled>},
    {"is_enabled", lua::wrap<l_is_enabled>},
    {"make_route", lua::wrap<l_make_route>},
    {"make_route_async", lua::wrap<l_make_route_async>},
    {"pull_route", lua::wrap<l_pull_route>},
    {"set_max_visited", lua::wrap<l_set_max_visited_blocks>},
    {"set_jump_height", lua::wrap<l_set_jump_height>},
    {"avoid_tag", lua::wrap<l_avoid_tag>},
    {NULL, NULL}
};
