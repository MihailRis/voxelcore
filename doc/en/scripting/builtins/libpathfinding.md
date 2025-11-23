# *pathfinding* library

The *pathfinding* library provides functions for working with the pathfinding system in the game world. It allows you to create and manage agents finding routes between points in the world.

When used in entity logic, the `core:pathfinding` component should be used.

## `core:pathfinding` component

```lua
local pf = entity:get_component("core:pathfinding")

--- ...
local x = ...
local y = ...
local z = ...

--- Set the target for the agent
pf.set_target({x, y, z})

--- Get the current target of the agent
local target = pf.get_target() --> vec3 or nil
--- ...

--- Get the current route of the agent
local route = pf.get_route() --> table<vec3> or nil
--- ...
```

## Library functions

```lua
--- Create a new agent. Returns the ID of the created agent
local agent = pathfinding.create_agent() --> int

--- Delete an agent by ID. Returns true if the agent existed, otherwise false
pathfinding.remove_agent(agent: int) --> bool

--- Set the agent state (enabled/disabled)
pathfinding.set_enabled(agent: int, enabled: bool)

--- Check the agent state. Returns true if the agent is enabled, otherwise false
pathfinding.is_enabled(agent: int) --> bool

--- Create a route based on the given points. Returns an array of route points
pathfinding.make_route(start: vec3, target: vec3) --> table<vec3>

--- Asynchronously create a route based on the given points.
--- This function allows to perform pathfinding in the background without blocking the main thread of execution
pathfinding.make_route_async(agent: int, start: vec3, target: vec3)

--- Get the route that the agent has already found. Used to get the route after an asynchronous search.
--- If the search has not yet completed, returns nil. If the route is not found, returns an empty table.
pathfinding.pull_route(agent: int) --> table<vec3> or nil

--- Set the maximum number of visited blocks for the agent. Used to limit the amount of work of the pathfinding algorithm.
pathfinding.set_max_visited(agent: int, max_visited: int)

--- Adding an avoided blocks tag
pathfinding.avoid_tag(
    agent: int,
    -- tag for avoided blocks
    tag: string, [optional],
    -- cost of crossing a block
    cost: int = 10
)
```
