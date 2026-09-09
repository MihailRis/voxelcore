local internals = __vc_internals

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

local this = {
    CH_TRANSLATE = 1,
    CH_ROTATE = 2,
    CH_SCALE = 3,
    CH_ZOOM = 4,

    INT_CONST = 1,
    INT_LINEAR = 2,
    INT_BEZIER = 3,
    INT_CUSTOM = 4,

    TRACE_CODEGEN = false,

    maths = {
        bezier_interpolation = bezier_interpolation
    }
}

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
local next_track_id = 1

function this.action(func)
    table.insert(running_actions, coroutine.create(func))
end

function this.play(name, target)
    local id = next_track_id
    table.insert(playing_tracks, {id=id, name=name, target=target, timer=0.0})
    next_track_id = id + 1
    return id
end

function this.stop(id)
    for i, entry in pairs(playing_tracks) do
        if entry.id == id then
            table.remove(playing_tracks, i)
            return
        end
    end
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
