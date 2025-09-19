local tsf = entity.transform
local body = entity.rigidbody
local rig = entity.skeleton
local mob = entity:require_component("core:mob")

local itemid = 0
local itemIndex = rig:index("item")

local function refresh_model(id)
    itemid = id
    rig:set_model(itemIndex, item.model_name(itemid))
    rig:set_matrix(itemIndex, mat4.rotate({0, 1, 0}, -80))
end

function on_render()
    local pid = entity:get_player()
    if pid == -1 then
        return
    end

    local rx, _, _ = player.get_rot(pid, pid ~= hud.get_player())

    local dir = vec2.rotate({0, -1}, -rx)
    mob.set_dir({dir[1], 0, dir[2]})

    local invid, slotid = player.get_inventory(pid)
    local id, _ = inventory.get(invid, slotid)
    if id ~= itemid then
        refresh_model(id)
    end
end

