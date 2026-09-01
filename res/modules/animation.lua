local this = {}

local CH_TRANSLATE = 1
local CH_ROTATE = 2

local INT_CONST = 1
local INT_LINEAR = 2
local INT_BEZIER = 3

local TRACE_CODEGEN = false

local DEFAULT_FPS = 60

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
        local t = (frame - left.frame) / (right.frame - left.frame)
        if interp == INT_BEZIER then
            return bezier_interpolation(left, right, t)
        end
        return left.value * (1.0 - t) + right.value * t
    end,
}
table.extend(env, math)

local function codegen_track(raw_track, lines, memoised, keysets)
    local code = ""
    local translation = {false, false, false}
    local rotation = {false, false, false}
    for i, line in ipairs(lines) do
        if line.expression then
            code = code .. "\n  local l" .. i .. " = (" ..
                process_expression(line.expression, memoised) .. ")"
        elseif line.keys then
            keysets[i] = line.keys
            code = code .. string.format(
                "\n  local l%d = value_at(keysets[%d], t * %s, %s)",
                i, i, raw_track.fps, line.interp)
        end

        if line.channel == CH_TRANSLATE then
            translation[line.axis] = i
        elseif line.channel == CH_ROTATE then
            rotation[line.axis] = i
        end
    end

    code = code .. "\n  mat4.idt(dst)"
    if translation[1] or translation[2] or translation[3] then
        code = code .. "\n  mat4.translate(dst, {" ..
        (translation[1] and ("l" .. translation[1]) or '0').. ", " ..
        (translation[2] and ("l" .. translation[2]) or '0').. ", " ..
        (translation[3] and ("l" .. translation[3]) or '0').. "}, dst)"
    end

    local axis_names = {"X", "Y", "Z"}
    for axis, var in ipairs(rotation) do
        if var then
            code = code .. "\n  mat4.rotate(dst, " .. axis_names[axis] ..
                ",l" ..  var .. ", dst)"
        end
    end

    return code
end

local action_to_channel = {
    move = CH_TRANSLATE,
    rotate = CH_ROTATE
}

local curve_to_interp = {
    const = INT_CONST,
    linear = INT_LINEAR,
    bezier = INT_BEZIER,
}

local function parse_configure(raw_track, node)
    if node.fps then
        raw_track.fps = node.fps
    end
    if node.frames then
        raw_track.duration = node.frames / node.fps
    elseif node.duration then
        raw_track.duration = node.duration
    end
end

local function parse_curve(line, node)
    line.interp = curve_to_interp[node.curve]
    line.keys = {}
    for j, key_node in ipairs(node) do
        local keyframe = {
            frame = tonumber(key_node.frame),
            value = tonumber(key_node.value),
        }
        if line.interp == INT_BEZIER then
            keyframe.lx = tonumber(key_node.lx)
            keyframe.ly = tonumber(key_node.ly)
            keyframe.rx = tonumber(key_node.rx)
            keyframe.ry = tonumber(key_node.ry)
        end
        table.insert(line.keys, keyframe)
    end
end

local function parse_track(root)
    local raw_track = {
        duration = math.huge,
        fps = DEFAULT_FPS,
        linesets = {},
    }
    local linesets = raw_track.linesets
    for i, node in ipairs(root) do
        if type(node) == "string" then
            goto continue
        end
        local tag = node['#']
        if tag == "configure" then
            parse_configure(raw_track, node)
            goto continue
        end

        local bone = node.bone

        local lineset = linesets[bone]
        if not lineset then
            lineset = {lines = {}}
            linesets[bone] = lineset
        end

        local line = {
            axis = ("xyz"):find(node.by),
            channel = action_to_channel[tag]
        }
        if node.func then
            line.expression = node.func
        elseif node.curve then
            parse_curve(line, node)
        else
            error("not implemented")
        end

        table.insert(lineset.lines, line)
        ::continue::
    end
    return raw_track
end


function this.compile_track(raw_track, track_name)
    local code = ""
    local memoised = {}
    local keysets = {}

    for bone, lineset in pairs(raw_track.linesets) do
        local lineset_code = codegen_track(
            raw_track, lineset.lines, memoised, keysets)

        code = code .. "\n do" .. lineset_code .. "\n end\n" ..
            " rig:set_matrix(rig:index(" .. string.escape(bone) .. "), dst)\n"
    end

    local memoised_code = ""
    for name, expression in pairs(memoised) do
        memoised_code = memoised_code .. "\n local " .. name .. " = "
            .. expression
    end

    if #memoised_code > 0 then
        code = memoised_code .. "\n" .. code
    end

    local src = "return function(rig, t, m)\n local dst = DST\n"
        .. code .. "\nend"

    if TRACE_CODEGEN then
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

local cached_tracks = {}

function this.load_vca(filepath, identifier)
    local track = cached_tracks[filepath]
    if track then
        return track
    end
    local source = file.read(filepath)
    local raw_track = parse_track(xml.parse_vcd(source, "track"))
    track = this.compile_track(raw_track, filepath)
    cached_tracks[identifier] = track
    return track
end

function this.get_track(identifier)
    return cached_tracks[identifier]
end

function __vc_internals.load_vca_animation(filepath, identifier)
    this.load_vca(filepath, identifier)
end

local running_actions = {}

function this.action(func)
    table.insert(running_actions, coroutine.create(func))
end

function __vc_internals.on_animation_frame()
    for i=#running_actions,1,-1 do
        local co = running_actions[i]
        local status, result = coroutine.resume(co)
        if not status then
            debug.error("error in animation action: "..result)
            table.remove(running_actions, i)
        elseif coroutine.status(co) == "dead" then
            table.remove(running_actions, i)
        end
    end
end

function __vc_internals.stop_all_actions()
    running_actions = {}
end

return this
