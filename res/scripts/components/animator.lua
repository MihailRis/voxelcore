local ALMOST_HUGE = 1e9

local rig = entity.skeleton

local states = {}

function create_state(name, track_id, max_intencity)
    states[name] = {
        track = nil,
        track_id = track_id,
        timer = 0.0,
        fade_time = 0.0,
        fade_timer = 0.0,
        fade_in = false,
        max_intencity = max_intencity,
    }
end

local current_state = nil

function set_state(name, fade_time)
    local state = states[name]
    if state == current_state then
        return
    end
    if current_state then
        current_state.fade_in = false
        current_state.fade_time = fade_time
        current_state.fade_timer = 0.0
    end
    current_state = state
    if state then
        state.timer = 0.0
        state.fade_in = true
        state.fade_time = fade_time
        state.fade_timer = 0.0
    end
end

local function update(delta)
    for _, state in pairs(states) do
        state.timer = state.timer + delta

        local intensity = state.fade_in and 1.0 or 0.0
        if state.fade_timer < state.fade_time then
            state.fade_timer = state.fade_timer + delta
            intensity = math.min(1.0, state.fade_timer / state.fade_time)
            if not state.fade_in then
                intensity = 1.0 - intensity
            end
        end

        state.track = state.track or animation.get_track(state.track_id)
        if state.track and intensity > 0.0 then
            state.track.func(
                rig,
                state.timer % math.min(ALMOST_HUGE, state.track.duration),
                intensity * state.max_intencity
            )
        end
    end
end

function on_render()
    update(time.delta())
end
