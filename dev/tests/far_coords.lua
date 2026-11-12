local util = require "core:tests_util"

util.create_demo_world("core:default")
app.set_setting("chunks.load-distance", 3)
app.set_setting("chunks.load-speed", 1)

local pid = player.create("Xerxes")

local function is_loaded(x, z)
    return block.get(math.floor(x), 0, math.floor(z)) ~= -1
end

local function wait_chunk(x, z)
    player.set_spawnpoint(pid, x, 100, z)
    player.set_pos(pid, x, 100, z)
    app.sleep_until(function () return is_loaded(x, z) end)
end

local function spawn_drop(x, y, z)
    return entities.spawn("base:drop", {x, y, z}, {base__drop={
        id=item.index("base:stone.item"),
        count=1
    }})
end

local function move_body(coord)
    local x, z = coord + 8.5, -coord - 8.5
    wait_chunk(x - 5, z)
    local entity = spawn_drop(x, 50.5, z)
    for _ = 1, 20 do
        entity.rigidbody:set_vel({3, 0, -2})
        app.tick()
    end
    local pos = entity.transform:get_pos()
    entity:despawn()
    return {pos[1] - x, pos[2] - 50.5, pos[3] - z}
end

local moved = move_body(0)
debug.print(moved)
assert(moved[1] > 1.0, "body didn't move")

for _, coord in ipairs({1000000, 30000000}) do
    local x, y, z = coord + 0.3, 100.6, -coord - 0.7

    player.set_pos(pid, x, y, z)
    util.check_vec3({x, y, z}, {player.get_pos(pid)}, "player.get_pos", 1e-6)

    player.set_spawnpoint(pid, x, y, z)
    util.check_vec3(
        {x, y, z}, {player.get_spawnpoint(pid)}, "player.get_spawnpoint", 1e-6
    )

    wait_chunk(x, z)

    local entity = spawn_drop(x, y, z)
    util.check_vec3({x, y, z}, entity.transform:get_pos(), "spawn pos", 1e-6)
    entity.transform:set_pos({x + 0.25, y, z - 0.25})
    util.check_vec3(
        {x + 0.25, y, z - 0.25}, entity.transform:get_pos(), "set_pos", 1e-6
    )
    entity:despawn()

    local ray = world.raycast({
        start = {x, 8, z},
        dir = {0, -1, 0},
        distance = 10,
    })
    assert(ray, "ray didn't hit")
    util.check_vec3({x, 1, z}, ray.endpoint, "ray.endpoint")
    util.check_vec3(
        {math.floor(x), 0, math.floor(z)}, ray.iendpoint, "ray.iendpoint"
    )
    util.check_float(7, ray.length, "ray.length")

    local far_moved = move_body(coord)
    debug.print(far_moved)
    util.check_vec3(moved, far_moved, "body displacement", 1e-3)
end

local x, z = 30000000.3, -30000000.7
player.set_spawnpoint(pid, x, 100.6, z)
player.set_pos(pid, x, 100.6, z)
app.close_world(true)
app.open_world("demo")

local px, _, pz = player.get_pos(pid)
util.check_float(x, px, "saved x", 1e-6)
util.check_float(z, pz, "saved z", 1e-6)
util.check_vec3(
    {x, 100.6, z}, {player.get_spawnpoint(pid)}, "saved spawnpoint", 1e-6
)
app.close_world(false)
app.delete_world("demo")
