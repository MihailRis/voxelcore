local target
local route
local started

local tsf = entity.transform
local body = entity.rigidbody

agent = pathfinding.create_agent()
pathfinding.set_max_visited(agent, 1e3)
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
            route = new_route
            started = false
        end
    end
end

function on_despawn()
    pathfinding.remove_agent(agent)
end
