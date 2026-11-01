local target
local route
local started

local tsf = entity.transform
local body = entity.rigidbody

local size = body:get_size()
agent = pathfinding.create_agent()
pathfinding.set_max_visited(agent, 1e3)
pathfinding.set_width(agent, math.max(size[1], size[3]))
pathfinding.set_height(agent, math.ceil(size[2]))
pathfinding.avoid_tag(agent, "core:liquid", 8)

function set_target(new_target)
    target = new_target
end

function set_jump_height(height)
    pathfinding.set_jump_height(agent, height)
end

function get_target()
    return target
end

function get_route()
    return route
end

function reset_route()
    route = nil
end

function next_waypoint()
    if not route or #route == 0 then
        return
    end
    local waypoint = route[#route]
    local pos = tsf:get_pos()
    local dst = vec2.length({
        math.floor(waypoint[1] - math.floor(pos[1])),
        math.floor(waypoint[3] - math.floor(pos[3]))
    })
    if dst < 1.0 then
        table.remove(route, #route)
    end
    return route[#route]
end

local function is_passed(waypoint, next_point, pos)
    local wx, wz = waypoint[1] + 0.5, waypoint[3] + 0.5
    local dx, dz = pos[1] - wx, pos[3] - wz
    if dx * dx + dz * dz > 2.25 then
        return false
    end
    return dx * (next_point[1] + 0.5 - wx) + dz * (next_point[3] + 0.5 - wz) > 0
end

local function drop_passed_waypoints(new_route)
    table.remove(new_route, #new_route)
    local pos = tsf:get_pos()
    while #new_route >= 2 and
        is_passed(new_route[#new_route], new_route[#new_route - 1], pos) do
        table.remove(new_route, #new_route)
    end
    return new_route
end

local refresh_internal = 100
local frameid = math.random(0, refresh_internal)

function set_refresh_interval(interval)
    refresh_internal = interval
end

function on_update()
    if not started then
        frameid = frameid + 1
        if body:is_grounded() then
            if target and (frameid % refresh_internal == 1 or not route) then
                pathfinding.make_route_async(agent, tsf:get_pos(), target)
                started = true
            end
        end
    else
        local new_route = pathfinding.pull_route(agent)
        if new_route then
            route = drop_passed_waypoints(new_route)
            started = false
        end
    end
end

function on_despawn()
    pathfinding.remove_agent(agent)
end
