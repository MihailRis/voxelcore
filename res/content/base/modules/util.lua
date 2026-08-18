local util = {}

util.DROP_FORCE = 8
util.DROP_INIT_VEL = { 0, 3, 0 }
util.DROP_MAX_ITEM_DIR_SHIFT_DEGREES = 65

---@param dir_shift? vec2 direction shift for drop; x and y must be clamped to [-0.5, 0.5]
local function calculate_item_drop_dir(pid, dir_shift)
    if not dir_shift then
        dir_shift = { 0, 0 }
    end
    local yaw, pitch = player.get_rot(pid)
    local q_yaw = quat.from_euler({ 0, yaw, 0 })
    local q_pitch = quat.from_euler({ pitch, 0, 0 })
    local q_rotation = quat.mul(q_yaw, q_pitch)

    local q_shift = quat.from_euler({
        -dir_shift[2] * util.DROP_MAX_ITEM_DIR_SHIFT_DEGREES * 2,
        -dir_shift[1] * util.DROP_MAX_ITEM_DIR_SHIFT_DEGREES * 2,
        0,
    })
    quat.mul(q_rotation, q_shift, q_rotation)
    return quat.mul_vec3(q_rotation, { 0, 0, -1 })
end

--- Create a drop from slot, set it velocity, and return it
---@param mode integer 0 - whole stack, 1 - single item, 2 - dupe whole stack
---@param dir_shift? vec2 direction shift for drop; x and y must be clamped to [-0.5, 0.5]
---@return table? entity, string? error
function util.drop_from_slot(pid, invid, slot, mode, dir_shift)
    if invid == 0 then
        return nil, "invid cannot be 0"
    end
    if mode == 2 and not player.is_infinite_items(pid) then
        return nil, "no permission to dupe items"
    end
    local itemid, itemcount = inventory.get(invid, slot)
    if itemid == 0 then
        return nil, "no item in slot"
    end

    local drop_itemcount = 1
    if mode == 0 then
        drop_itemcount = itemcount
    end
    if mode == 2 then
        drop_itemcount = item.stack_size(itemid)
    else
        inventory.set(invid, slot, itemid, itemcount - drop_itemcount)
    end

    local data = inventory.get_all_data(invid, slot)
    local pvel = { player.get_vel(pid) }
    local ppos = vec3.add({ player.get_pos(pid) }, { 0, 0.7, 0 })
    local dir = calculate_item_drop_dir(pid, dir_shift)
    local throw_force = vec3.mul(dir, util.DROP_FORCE)
    local drop, err = util.drop(ppos, itemid, drop_itemcount, data, 1.5)
    if not drop then
        return nil, err
    end
    local velocity = vec3.add(throw_force, vec3.add(pvel, util.DROP_INIT_VEL))
    drop.rigidbody:set_vel(velocity)
    return drop
end

---@return table? entity, string? error
function util.drop(ppos, itemid, count, data, pickup_delay)
    if itemid == 0 or not itemid then
        return nil, "item is empty"
    end
    return entities.spawn("base:drop", ppos, {
        base__drop = {
            id = itemid,
            count = count,
            data = data,
            pickup_delay = pickup_delay
        }
    })
end

function util.calc_loot(loot_table)
    local results = {}
    for _, loot in ipairs(loot_table) do
        local chance = loot.chance or 1
        local count = loot.count or 1

        local roll = math.random()

        if roll < chance then
            if loot.min and loot.max then
                count = math.random(loot.min, loot.max)
            end
            if count == 0 then
                goto continue
            end
            table.insert(results, {
                item = item.index(loot.item), count = count
            })
        end
        ::continue::
    end
    return results
end

function util.block_loot(blockid)
    local lootscheme = block.properties[blockid]["base:loot"]
    if lootscheme then
        return util.calc_loot(lootscheme)
    end
    return { { item = block.get_picking_item(blockid), count = 1 } }
end

return util
