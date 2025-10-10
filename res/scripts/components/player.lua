local tsf = entity.transform
local body = entity.rigidbody
local mob = entity:require_component("core:mob")

local cheat_speed_mul = 10.0

local function get_player_rotation(pid)
    local rx, ry, rz = player.get_rot(pid)
    local matrix = mat4.rotate({0, 1, 0}, rx)
    mat4.rotate(matrix, {1, 0, 0}, ry, matrix)
    mat4.rotate(matrix, {0, 0, 1}, rz, matrix)
    return matrix
end

local function process_player_inputs(pid, rot, delta)
    if not hud or hud.is_inventory_open() or menu.page ~= "" then
        return
    end

    local front = mat4.mul(rot, {0, 0, -1})
    local right = mat4.mul(rot, {1, 0, 0})

    front[2] = 0.0
    vec3.normalize(front, front)

    local isjump = input.is_active('movement.jump')
    local issprint = input.is_active('movement.sprint')
    local iscrouch = input.is_active('movement.crouch')
    local isforward = input.is_active('movement.forward')
    local ischeat = input.is_active('movement.cheat')
    local isback = input.is_active('movement.back')
    local isleft = input.is_active('movement.left')
    local isright = input.is_active('movement.right')
    body:set_crouching(iscrouch)

    local vel = body:get_vel()
    local speed = ischeat and cheat_speed_mul or 1.0

    local dir = {0, 0, 0}

    if isforward then vec3.add(dir, front, dir) end
    if isback then vec3.sub(dir, front, dir) end
    if isright then vec3.add(dir, right, dir) end
    if isleft then vec3.sub(dir, right, dir) end

    if vec3.length(dir) > 0.0 then
        mob.go({dir[1], dir[3]}, speed, issprint, iscrouch, vel)
    end

    if mob.is_flight() then
        if isjump then
            mob.move_vertical(speed * 3)
        elseif iscrouch then
            mob.move_vertical(-speed * 3)
        end
    elseif body:is_grounded() and isjump then
        mob.jump()
    end
end

function on_physics_update(delta)
    local pid = entity:get_player()
    if pid == -1 then
        return
    end

    mob.set_flight(player.is_flight(pid))
    body:set_body_type(player.is_noclip(pid) and "kinematic" or "dynamic")

    local rot = get_player_rotation(pid)
    local front = mat4.mul(rot, {0, 0, -1})
    local pos = tsf:get_pos()

    if hud and pid == hud.get_player() then
        process_player_inputs(pid, rot, delta)
    end
    mob.look_at(vec3.add(pos, front))
end
