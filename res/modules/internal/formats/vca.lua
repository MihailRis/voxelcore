local internals = __vc_internals

local DEFAULT_FPS = 60
local INT_BEZIER = animation.INT_BEZIER

local action_to_channel = {
    move = animation.CH_TRANSLATE,
    rotate = animation.CH_ROTATE,
    scale = animation.CH_SCALE,
    zoom = animation.CH_ZOOM,
}

local curve_to_interp = {
    const = animation.INT_CONST,
    linear = animation.INT_LINEAR,
    bezier = animation.INT_BEZIER,
}

local function parse_configure(raw_track, node)
    if node.fps then
        raw_track.fps = node.fps
    end
    if node.frames then
        raw_track.duration = node.frames / (node.fps or DEFAULT_FPS)
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

        local target_type = nil
        if node.bone then
            target_type = "bone"
        end
        local target_name = node.bone or ""
        local lineset = linesets[target_name]
        if not lineset then
            lineset = {
                lines = {},
                target_type = target_type,
                target_name = target_name
            }
            linesets[target_name] = lineset
        end

        local line = {
            axis = node.by and ("xyz"):find(node.by) or "",
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
    debug.print(raw_track)
    return raw_track
end

local function load_vca(filepath)
    local source = file.read(filepath)
    local raw_track = parse_track(xml.parse_vcd(source, "track"))
    debug.print(raw_track)
    return animation.compile_track(raw_track, filepath)
end

function internals.load_vca_animation(filepath, identifier)
    local track = load_vca(filepath)
    internals.store_animation(identifier, track)
end
