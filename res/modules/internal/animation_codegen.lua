local internals = __vc_internals

local INT_CONST = animation.INT_CONST
local INT_BEZIER = animation.INT_BEZIER

local bezier_interpolation = animation.maths.bezier_interpolation

local patterns =  {}
local exclude_patters = {
    "end",
    (string.pattern_safe("'")),
    (string.pattern_safe('"')),
    (string.pattern_safe("--")),
    (string.pattern_safe("..")),
}

--  TODO: replace with actual expression -> lua translator
local function process_expression(src, memoised)
    for i, pattern in ipairs(exclude_patters) do
        if src:find(pattern) then
            debug.print(exclude_patters)
            error("invalid syntax "..string.escape(src))
        end
    end
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
        elseif type(interp) == "function" then
            return interp(left.value, right.value, t)
        end
        return left.value * (1.0 - t) + right.value * t
    end,
    value_at_custom = function(keys, frame, func)
        local left, right = key_neighbors(keys, frame)
        if left == right then
            return keys[left].value
        end
        left = keys[left]
        right = keys[right]
        if left == nil then
            return right.value
        end
        local t = (frame - left.frame) / (right.frame - left.frame)
        return func(left, right, t)
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
            local target_keysets = keysets[lineset.target_name]
            if not target_keysets then
                target_keysets = {}
                keysets[lineset.target_name] = target_keysets
            end
            target_keysets[i] = line.keys

            if line.curve_func then
                local valueat = string.format("curves[%s]", string.escape(line.curve_func))

                code = code .. string.format(
                "\n   local l%d = value_at_custom(keysets['%s'][%d], t * %s, %s)",
                i, lineset.target_name, i, raw_track.fps, valueat)
            else
                code = code .. string.format(
                    "\n   local l%d = value_at(keysets['%s'][%d], t * %s, %s)",
                    i, lineset.target_name, i, raw_track.fps, line.interp)
            end
        end

        if line.channel == animation.CH_TRANSLATE then
            translation[line.axis] = i
            has_tsf = true
        elseif line.channel == animation.CH_ROTATE then
            rotation[line.axis] = i
            has_tsf = true
        elseif line.channel == animation.CH_SCALE then
            scale[line.axis] = i
            has_tsf = true
        elseif line.channel == animation.CH_ZOOM then
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
    local rotation_order = raw_track.rotation_order or "XYZ"
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

local function codegen_rig_target(raw_track, context)
    local code = "\n if target.set_matrix and target.index then\n"
    code = code .. "  local dst = DST\n"
    for bone, lineset in pairs(raw_track.linesets) do
        if lineset.target_type ~= "bone" then
            goto continue
        end
        local lineset_code = codegen_track(
            raw_track, lineset, context.memoised, context.keysets, true)

        code = code .. "\n  do" .. lineset_code .. "\n  end\n" ..
            "  target:set_matrix(target:index(" .. string.escape(bone) .. "), dst)\n"
        ::continue::
    end
    return code .. " end"
end

local function codegen_object_target(raw_track, context)
    local code = "\n if target.set_pos then\n"
    code = code .. "  local dst = DST\n"
    local lineset = raw_track.linesets[""]
    if not lineset then
        return ""
    end
    local lineset_code = codegen_track(
        raw_track, lineset, context.memoised, context.keysets, true)
    code = code .. "\n  do" .. lineset_code .. "\n  end\n"
    .. "  set_matrix(target, dst)\n"
    return code .. " end"
end

local function codegen_camera_target(raw_track, context)
    local code = "\n if target.set_zoom then\n"
    code = code .. "  local zoom = 1.0\n"
    local lineset = raw_track.linesets[""]
    if not lineset then
        return ""
    end
    local lineset_code = codegen_track(
        raw_track, lineset, context.memoised, context.keysets, false)
    code = code .. "\n  do" .. lineset_code .. "\n  end\n"
    .. "  target:set_zoom(zoom)\n"
    return code .. " end"
end


function internals.compile_animation_track(raw_track, track_name)
    local code = ""
    local context = {
        memoised = {},
        keysets = {},
        curves = {},
    }
    for name, curve in pairs(raw_track.curves) do
        context.curves[name] = load(string.format(
            "return function(kl, kr, t) return %s end",
            process_expression(curve.func, context.memoised)
        ), "<curve>", "t", env)()
    end

    code = code .. codegen_rig_target(raw_track, context)
    code = code .. codegen_object_target(raw_track, context)
    code = code .. codegen_camera_target(raw_track, context)

    local memoised_code = ""
    for name, expression in pairs(context.memoised) do
        memoised_code = memoised_code .. "\n local " .. name .. " = "
            .. expression
    end

    if #memoised_code > 0 then
        code = memoised_code .. "\n" .. code
    end

    local src = "return function(target, t, m)\n"
        .. code .. "\nend"

    if animation.TRACE_CODEGEN then
        debug.log("["..
            string.escape(track_name or "nil").." codegen trace]:\n"..src)
    end

    local generator, err = load(
        src, "<expr>", "bt", table.extend({
            keysets = context.keysets, curves = context.curves
        }, env))
    if not generator then
        error(err)
    end
    return {
        duration = raw_track.duration,
        func = generator(),
    }
end
