local tsf = entity.transform
local body = entity.rigidbody

local blockid = ARGS.block
local blockstates = ARGS.states or 0
if SAVED_DATA.block then
    blockid = SAVED_DATA.block
    blockstates = SAVED_DATA.states or 0
else
    SAVED_DATA.block = blockid
    SAVED_DATA.states = blockstates
end

if vc.is_client() then-- setup visuals
    local rig = entity.skeleton
    local id = block.index(blockid)
    local rotation = block.decompose_state(blockstates)[1]
    local textures = block.get_textures(id)
    for i,t in ipairs(textures) do
        rig:set_texture("$"..tostring(i-1), "blocks:"..textures[i])
    end
    local axisX = {block.get_X(id, rotation)}
    local axisY = {block.get_Y(id, rotation)}
    local axisZ = {block.get_Z(id, rotation)}
    local matrix = {
        axisX[1], axisX[2], axisX[3], 0,
        axisY[1], axisY[2], axisY[3], 0,
        axisZ[1], axisZ[2], axisZ[3], 0,
        0, 0, 0, 1
    }
    rig:set_matrix(0, matrix)
    body:set_material(block.material(id))
end

function on_grounded()
    local pos = tsf:get_pos()
    local ix = math.floor(pos[1])
    local iy = math.floor(pos[2])
    local iz = math.floor(pos[3])
    if block.is_replaceable_at(ix, iy, iz) then
        block.place(ix, iy, iz, block.index(blockid), blockstates)
    else
        local picking_item = block.get_picking_item(block.index(blockid))
        local drop = entities.spawn("base:drop", pos, {base__drop={id=picking_item, count=1}})
        drop.rigidbody:set_vel(vec3.spherical_rand(5.0))
    end
    entity:despawn()
end
