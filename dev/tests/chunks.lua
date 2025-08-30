local util = require "core:tests_util"
util.create_demo_world()

app.set_setting("chunks.load-distance", 15)
app.set_setting("chunks.load-speed", 15)

local pid1 = player.create("Xerxes")
assert(player.get_name(pid1) == "Xerxes")

local pid2 = player.create("Segfault")
assert(player.get_name(pid2) == "Segfault")

local seed = math.floor(math.random() * 1e6)
print("random seed", seed)
math.randomseed(seed)

for i=1,25 do
    if i % 5 == 0 then
        print(tostring(i*4).." % done")
        print("chunks loaded", world.count_chunks())
    end
    player.set_pos(pid1, math.random() * 100 - 50, 100, math.random() * 100 - 50)
    player.set_pos(pid2, math.random() * 200 - 100, 100, math.random() * 200 - 100)
    app.tick()
end

player.delete(pid2)

app.close_world(true)
app.delete_world("demo")
