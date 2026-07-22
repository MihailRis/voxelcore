local util = require "core:tests_util"

-- Create world and prepare settings
util.create_demo_world("core:default")
app.set_setting("chunks.load-distance", 3)
app.set_setting("chunks.load-speed", 1)

-- Create player
local pid = player.create("Xerxes")
player.set_spawnpoint(pid, 0, 100, 0)
player.set_pos(pid, 0, 100, 0)
app.sleep_until(function () return block.get(0, 0, 0) ~= -1 end)
app.sleep_until(function () return block.get(-5, 8, 2) ~= -1 end)

local sand_idx = block.index("base:sand")
block.set(0, 1, 0, sand_idx)

local start = {-5, 8, 2}
local target = {0.5, 1.5, 0.5}
local ray = world.raycast({
    start = start,
    dir = vec3.normalize(vec3.sub(target, start)),
    distance = 10,
})
debug.print(ray)

assert(ray, "ray didn't hit")
util.check_int(sand_idx, ray.block, "ray.block")
util.check_vec3({0, 1, 0}, ray.normal, "ray.normal")
util.check_vec3({0.0769, 2, 0.6153}, ray.endpoint, "ray.endpoint")
util.check_vec3({0, 1, 0}, ray.iendpoint, "ray.iendpoint")
util.check_float(7.9807, ray.length, "ray.length")

ray = world.raycast({
    start = start,
    dir = vec3.normalize(vec3.sub(target, start)),
    distance = 10,
    filter_blocks = {sand_idx},
})
debug.print(ray)

assert(ray, "ray didn't hit")
util.check_int(block.index("core:obstacle"), ray.block, "ray.block")
util.check_vec3({0, 1, 0}, ray.normal, "ray.normal")
util.check_vec3({0.923, 1, 0.3846}, ray.endpoint, "ray.endpoint")
util.check_vec3({0, 0, 0}, ray.iendpoint, "ray.iendpoint")
util.check_float(9.3108, ray.length, "ray.length")
