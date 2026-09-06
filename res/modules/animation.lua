local internals = __vc_internals

local this = {
    CH_TRANSLATE = 1,
    CH_ROTATE = 2,
    CH_SCALE = 3,
    CH_ZOOM = 4,

    INT_CONST = 1,
    INT_LINEAR = 2,
    INT_BEZIER = 3,

    TRACE_CODEGEN = true,
}

local INT_CONST = this.INT_CONST
local INT_BEZIER = this.INT_BEZIER

local function bezier(a, b, c, d, u)
    local s = 1 - u
    return s * s * s * a +
        3 * s * s * u * b +
        3 * s * u * u * c +
        u * u * u * d
end

local function bezier_derivative(a, b, c, d, u)
    local s = 1 - u
    return
        3 * s * s * (b - a) +
        6 * s * u * (c - b) +
        3 * u * u * (d - c)
end

local function bezier_interpolation(k0, k1, t)
    local frame = k0.frame + t * (k1.frame - k0.frame)
    local u = t

    for i=1,8 do
        local x = bezier(k0.frame, k0.rx, k1.lx, k1.frame, u)
        local dx = bezier_derivative(k0.frame, k0.rx, k1.lx, k1.frame, u)

        if math.abs(dx) < 1e-8 then
            break
        end

        u = u - (x - frame) / dx

        if u < 0 then u = 0 end
        if u > 1 then u = 1 end
    end

    return bezier(k0.value, k0.ry, k1.ly, k1.value, u)
end

local patterns =  {
    {name="sint", pattern="sin(t)"},
    {name="sint2", pattern="sin(t * 2)"},
}

local function process_expression(src, memoised)
    for i, pattern in ipairs(patterns) do
        local pattern_safe = string.pattern_safe(pattern.pattern)
        if src:find(pattern_safe) then
            memoised[pattern.name] = pattern.pattern
            src = src:gsub(pattern_safe, pattern.name)
        end
    end
    return src
end

local function key_neighbors(keys, frame)
    local left = 1
    local right = #keys

    while left <= right do
        local mid = math.floor((left + right) / 2)

        if keys[mid].frame < frame then
            left = mid + 1
        elseif keys[mid].frame > frame then
            right = mid - 1
        else
            return mid, mid
        end
    end
    if left > #keys then
        left = #keys
    end
    return right, left
end

local env = {
    mat4 = mat4,
    e = math.exp(1),
    X = {1, 0, 0},
    Y = {0, 1, 0},
    Z = {0, 0, 1},
    DST = mat4.idt(),
    value_at = function(keys, frame, interp)
        local left, right = key_neighbors(keys, frame)
        if left == right then
            return keys[left].value
        end
        left = keys[left]
        if interp == INT_CONST then
            return left.value
        end
        right = keys[right]
        if left == nil then
            return right.value
        end
        local t = (frame - left.frame) / (right.frame - left.frame)
        if interp == INT_BEZIER then
            return bezier_interpolation(left, right, t)
        end
        return left.value * (1.0 - t) + right.value * t
    end,
    set_matrix = function(target, matrix)
        local info = mat4.decompose(matrix)
        if info then
            local pos = info.translation
            local rot = info.rotation
            local scale = info.scale
            if target.set_pos then
                target:set_pos(pos)
            end
            if target.set_rot then
                target:set_rot(rot)
            end
            if target.set_scale then
                target:set_scale(scale)
            end
        end
    end,
    dump = debug.print
}

local math_funcs = {
    "sqrt", "min", "max", "deg", "rad", "log", "log10", "floor", "ceil", "sin",
    "tan", "noise", "noise2", "sign", "round", "exp", "pi", "e"
}
for _, name in ipairs(math_funcs) do
    env[name] = math[name]
end

local function codegen_track(raw_track, lineset, memoised, keysets, use_tsf)
    local lines = lineset.lines
    local code = ""
    local has_tsf = false
    local translation = {false, false, false}
    local rotation = {false, false, false}
    local scale = {false, false, false}
    for i, line in ipairs(lines) do
        if line.expression then
            code = code .. "\n   local l" .. i .. " = (" ..
                process_expression(line.expression, memoised) .. ")"
        elseif line.keys then
            debug.print(raw_track)
            local target_keysets = keysets[lineset.target_name]
            if not target_keysets then
                target_keysets = {}
                keysets[lineset.target_name] = target_keysets
            end
            target_keysets[i] = line.keys
            code = code .. string.format(
                "\n   local l%d = value_at(keysets['%s'][%d], t * %s, %s)",
                i, lineset.target_name, i, raw_track.fps, line.interp)
        end

        if line.channel == this.CH_TRANSLATE then
            translation[line.axis] = i
            has_tsf = true
        elseif line.channel == this.CH_ROTATE then
            rotation[line.axis] = i
            has_tsf = true
        elseif line.channel == this.CH_SCALE then
            scale[line.axis] = i
            has_tsf = true
        elseif line.channel == this.CH_ZOOM then
            code = code .. "\n  zoom = l" .. i
        end
    end

    if not has_tsf or not use_tsf then
        return code
    end

    code = code .. "\n   mat4.idt(dst)"
    if translation[1] or translation[2] or translation[3] then
        code = code .. "\n   mat4.translate(dst, {" ..
        (translation[1] and ("l" .. translation[1]) or '0').. ", " ..
        (translation[2] and ("l" .. translation[2]) or '0').. ", " ..
        (translation[3] and ("l" .. translation[3]) or '0').. "}, dst)"
    end

    local axis_names = {"X", "Y", "Z"}
    local axis_indices = {X=1, Y=2, Z=3}
    local rotation_order = raw_track.rotation_order
    for i=1,3 do
        local axis = axis_indices[rotation_order[i]]
        local var = rotation[axis]
        if var then
            code = code .. "\n   mat4.rotate(dst, " .. axis_names[axis] ..
                ", l" ..  var .. ", dst)"
        end
    end

    if scale[1] or scale[2] or scale[3] then
        code = code .. "\n   mat4.scale(dst, {" ..
        (scale[1] and ("l" .. scale[1]) or '1').. ", " ..
        (scale[2] and ("l" .. scale[2]) or '1').. ", " ..
        (scale[3] and ("l" .. scale[3]) or '1').. "}, dst)"
    end

    return code
end

local function codegen_rig_target(raw_track, memoised, keysets)
    local code = "\n if target.set_matrix and target.index then\n"
    code = code .. "  local dst = DST\n"
    for bone, lineset in pairs(raw_track.linesets) do
        if lineset.target_type ~= "bone" then
            goto continue
        end
        local lineset_code = codegen_track(
            raw_track, lineset, memoised, keysets, true)

        code = code .. "\n  do" .. lineset_code .. "\n  end\n" ..
            "  target:set_matrix(target:index(" .. string.escape(bone) .. "), dst)\n"
        ::continue::
    end
    return code .. " end"
end

local function codegen_object_target(raw_track, memoised, keysets)
    local code = "\n if target.set_pos then\n"
    code = code .. "  local dst = DST\n"
    local lineset = raw_track.linesets[""]
    if not lineset then
        return ""
    end
    local lineset_code = codegen_track(
        raw_track, lineset, memoised, keysets, true)
    code = code .. "\n  do" .. lineset_code .. "\n  end\n"
    .. "  set_matrix(target, dst)\n"
    return code .. " end"
end

local function codegen_camera_target(raw_track, memoised, keysets)
    local code = "\n if target.set_zoom then\n"
    code = code .. "  local zoom = 1.0\n"
    local lineset = raw_track.linesets[""]
    if not lineset then
        return ""
    end
    local lineset_code = codegen_track(
        raw_track, lineset, memoised, keysets, false)
    code = code .. "\n  do" .. lineset_code .. "\n  end\n"
    .. "  target:set_zoom(zoom)\n"
    return code .. " end"
end

function this.compile_track(raw_track, track_name)
    local code = ""
    local memoised = {}
    local keysets = {}

    code = code .. codegen_rig_target(raw_track, memoised, keysets)
    code = code .. codegen_object_target(raw_track, memoised, keysets)
    code = code .. codegen_camera_target(raw_track, memoised, keysets)

    local memoised_code = ""
    for name, expression in pairs(memoised) do
        memoised_code = memoised_code .. "\n local " .. name .. " = "
            .. expression
    end

    if #memoised_code > 0 then
        code = memoised_code .. "\n" .. code
    end

    local src = "return function(target, t, m)\n"
        .. code .. "\nend"

    if this.TRACE_CODEGEN then
        debug.log("["..
            string.escape(track_name or "nil").." codegen trace]:\n"..src)
    end

    local generator, err = load(
        src, "<expr>", "bt", table.extend({keysets = keysets}, env))
    if not generator then
        error(err)
    end
    return {
        duration = raw_track.duration,
        func = generator(),
    }
end

local loaded_tracks = {}
local backup_tracks = {}

function internals.store_animation(name, track)
    loaded_tracks[name] = track
end

function this.get_track(identifier)
    return loaded_tracks[identifier]
end

local running_actions = {}
local playing_tracks = {}

function this.action(func)
    table.insert(running_actions, coroutine.create(func))
end

function this.play(name, target)
    table.insert(playing_tracks, {name=name, target=target, timer=0.0})
end

function internals.on_animation_frame()
    for i=1,#running_actions do
        local co = running_actions[i]
        local status, result = coroutine.resume(co)
        if not status then
            debug.error("error in animation action: "..result)
            table.remove(running_actions, i)
        elseif coroutine.status(co) == "dead" then
            table.remove(running_actions, i)
        end
    end
    local delta = time.delta()
    for i=1,#playing_tracks do
        local track_info = playing_tracks[i]
        local track = loaded_tracks[track_info.name]
        if not track then
            debug.error("animation track not found: "..track_info.name)
            table.remove(playing_tracks, i)
        else
            track_info.timer = track_info.timer + delta
            if track_info.timer > track.duration then
                table.remove(playing_tracks, i)
            else
                track.func(track_info.target, track_info.timer)
            end
        end
    end
end

function internals.stop_animation_actions()
    running_actions = {}
end

function internals.backup_and_clear_animation()
    loaded_tracks, backup_tracks = backup_tracks, {}
end

function internals.restore_animation_backup()
    loaded_tracks, backup_tracks = backup_tracks, {}
end

return this
