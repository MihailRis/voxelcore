local internals = __vc_internals

local DEFAULT_FPS = 60

local action_to_channel = {
    move = animation.CH_TRANSLATE,
    rotate = animation.CH_ROTATE,
    scale = animation.CH_SCALE,
    zoom = animation.CH_ZOOM,
    texture = animation.CH_TEXTURE,
    show = animation.CH_SHOW,
    model = animation.CH_MODEL,
    color = animation.CH_COLOR,
}

local curve_to_interp = {
    const = animation.INT_CONST,
    linear = animation.INT_LINEAR,
    bezier = animation.INT_BEZIER,
}

local boolean_actions = {
    show = true
}

local function parse_configure(raw_track, node)
    if node.fps then
        raw_track.fps = tonumber(node.fps)
    end
    if node.frames then
        raw_track.duration = tonumber(node.frames) / (tonumber(raw_track.fps) or DEFAULT_FPS)
    elseif node.duration then
        raw_track.duration = tonumber(node.duration)
    end
    raw_track.rotation_order = string.upper(node["rotation-order"] or "XYZ")
end

local function parse_curve(line, node)
    if node.curve:starts_with(".") then
        line.interp = animation.INT_CUSTOM
        line.curve_func = node.curve:sub(2)
    else
        line.interp = curve_to_interp[node.curve]
    end
    line.keys = {}
    for j, key_node in ipairs(node) do
        local keyframe = {
            frame = tonumber(key_node.frame),
            value = tonumber(key_node.value),
        }
        if line.interp == animation.INT_BEZIER then
            keyframe.lx = tonumber(key_node.lx)
            keyframe.ly = tonumber(key_node.ly)
            keyframe.rx = tonumber(key_node.rx)
            keyframe.ry = tonumber(key_node.ry)
        end
        table.insert(line.keys, keyframe)
    end
end

local function parse_simple_frames(line, node)
    line.keys = {}
    for j, key_node in ipairs(node) do
        local keyframe = {
            frame = tonumber(key_node.frame),
            value = key_node.value,
        }
        table.insert(line.keys, keyframe)
    end
end

local function parse_boolean_frames(line, node)
    line.keys = {}
    for j, key_node in ipairs(node) do
        local keyframe = {
            frame = tonumber(key_node.frame),
            value = key_node.value == "on",
        }
        table.insert(line.keys, keyframe)
    end
end

local function parse_directive(node, raw_track)
    local linesets = raw_track.linesets
    local tag = node['#']
    if tag == "configure" then
        parse_configure(raw_track, node)
        return
    elseif tag == "curve" then
        raw_track.curves[node.name] = node
        return
    end

    local target_type = nil
    if node.bone then
        target_type = "bone"
    elseif node.zoom then
        target_type = "camera"
    elseif tag == "texture" then
        target_type = "texture"
    end
    local target_name = node.bone or node.name or ""
    local lineset = linesets[target_name]
    if not lineset then
        lineset = {
            lines = {},
            target_type = target_type,
            target_name = target_name,
            flag = node.flag,
        }
        linesets[target_name] = lineset
    end

    local channel = action_to_channel[tag]
    if not channel then
        error("unknown directive " .. tag:escape())
    end
    local line = {
        axis = node.by and (("xyz"):find(node.by) or ("rgba"):find(node.by)) or "",
        channel = channel,
        period = node.period or animation.MAX_FRAMES
    }
    if node.func then
        line.expression = node.func
    elseif node.curve then
        parse_curve(line, node)
    elseif boolean_actions[tag] then
        parse_boolean_frames(line, node)
    else
        parse_simple_frames(line, node)
    end

    table.insert(lineset.lines, line)
end

local function parse_track(root)
    local raw_track = {
        duration = math.huge,
        fps = DEFAULT_FPS,
        linesets = {},
        curves = {},
    }
    for i, node in ipairs(root) do
        if type(node) == 'table' then
            parse_directive(node, raw_track)
        end
    end
    return raw_track
end

local function load_vca(source, filepath)
    local raw_track = parse_track(xml.parse_vcd(source, "track"))
    return internals.compile_animation_track(raw_track, filepath)
end

function internals.load_vca_animation(filepath, source, identifier)
    local track = load_vca(source or file.read(filepath), filepath)
    internals.store_animation(identifier, track)
end
