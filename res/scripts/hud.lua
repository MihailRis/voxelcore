local function configure_SSAO()
    -- Temporary using slot to configure built-in SSAO effect
    local slot = gfx.posteffects.index("core:default")
    gfx.posteffects.set_effect(slot, "ssao")

    -- Generating random SSAO samples
    local buffer = Bytearray(0)
    for i = 0, 63 do
        local x = math.random() * 2.0 - 1.0
        local y = math.random() * 2.0 - 1.0
        local z = math.random() * 2.0
        local len = math.sqrt(x * x + y * y + z * z)
        if len > 0 then
            x = x / len
            y = y / len
            z = z / len
        end
        Bytearray.append(buffer, byteutil.pack("fff", x, y, z))
    end
    gfx.posteffects.set_array(slot, "u_ssaoSamples", Bytearray_as_string(buffer))
    -- SSAO effect configured, so 'core:default' slot may be reused now 
    -- for test purposes
end

local function update_hand()
    local skeleton = gfx.skeletons
    local pid = hud.get_player()
    local invid, slot = player.get_inventory(pid)
    local itemid = inventory.get(invid, slot)

    local cam = cameras.get("core:first-person")
    local bone = skeleton.index("hand", "item")

    local offset = vec3.mul(vec3.sub(cam:get_pos(), {player.get_pos(pid)}), -1)

    local rotation = cam:get_rot()

    local angle = player.get_rot(pid) - 90
    local cos = math.cos(angle / (180 / math.pi))
    local sin = math.sin(angle / (180 / math.pi))

    local newX = offset[1] * cos - offset[3] * sin
    local newZ = offset[1] * sin + offset[3] * cos

    offset[1] = newX
    offset[3] = newZ

    local mat = mat4.translate(mat4.idt(), {0.06, 0.035, -0.1})
    mat4.scale(mat, {0.1, 0.1, 0.1}, mat)
    mat4.mul(rotation, mat, mat)
    mat4.rotate(mat, {0, 1, 0}, -90, mat)
    mat4.translate(mat, offset, mat)

    skeleton.set_matrix("hand", bone, mat)
    skeleton.set_model("hand", bone, item.model_name(itemid))
end

function on_hud_open()
    input.add_callback("player.pick", function ()
        if hud.is_paused() or hud.is_inventory_open() then
            return
        end
        local pid = hud.get_player()
        local x, y, z = player.get_selected_block(pid)
        if x == nil then
            return
        end
        local id = block.get_picking_item(block.get(x, y, z))
        local inv, cur_slot = player.get_inventory(pid)
        local slot = inventory.find_by_item(inv, id, 0, 9)
        if slot then
            player.set_selected_slot(pid, slot)
            return
        end
        if not rules.get("allow-content-access") then
            return
        end
        slot = inventory.find_by_item(inv, 0, 0, 9)
        if slot then
            cur_slot = slot
        end
        player.set_selected_slot(pid, cur_slot)
        inventory.set(inv, cur_slot, id, 1)
    end)

    input.add_callback("player.noclip", function ()
        if hud.is_paused() or hud.is_inventory_open() then
            return
        end
        local pid = hud.get_player()
        if player.is_noclip(pid) then
            player.set_flight(pid, false)
            player.set_noclip(pid, false)
        else
            player.set_flight(pid, true)
            player.set_noclip(pid, true)
        end
    end)

    input.add_callback("player.flight", function ()
        if hud.is_paused() or hud.is_inventory_open() then
            return
        end
        local pid = hud.get_player()
        if player.is_noclip(pid) then
            return
        end
        if player.is_flight(pid) then
            player.set_flight(pid, false)
        else
            player.set_flight(pid, true)
            player.set_vel(pid, 0, 1, 0)
        end
    end)

    configure_SSAO()

    hud.default_hand_controller = update_hand
end

function on_hud_render()
    if hud.hand_controller then
        hud.hand_controller()
    else
        update_hand()
    end
end
