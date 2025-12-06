local initialized = false

-- FPS tracking state
local fps = 0
local fpsMin = 0
local fpsMax = 0

-- Network throughput tracking (bytes per second)
local lastTotalDownload = 0
local lastTotalUpload = 0
local netSpeedString = "net: download: - B/s upload: - B/s"

local function bool_to_onoff(v)
    if v == nil then return "-" end
    if v then return "on" else return "off" end
end

-- Convert integer to binary string with fixed width
local function to_bin(n, bits)
    n = tonumber(n) or 0
    if bits == nil or bits <= 0 then bits = 1 end
    local s = ""
    for i = bits - 1, 0, -1 do
        local bit = (math.floor(n / (2 ^ i)) % 2)
        s = s .. tostring(bit)
    end
    return s
end

local function update_fps_minmax()
    local dt = time.delta()
    if dt <= 0 then dt = 0.000001 end
    fps = math.floor(1.0 / dt + 0.5)
    if fpsMin == 0 or fps < fpsMin then fpsMin = fps end
    if fpsMax == 0 or fps > fpsMax then fpsMax = fps end
end

local function update_fps_text()
    document.dbg_fps.text = string.format("FPS: %d/%d", fpsMax, fpsMin)
    fpsMin = fps
    fpsMax = fps
end

local function update_debug_info()
    -- Audio counters
    local spk = audio.count_speakers()
    local strm = audio.count_streams()
    document.dbg_audio.text = string.format("audio: speakers: %d streams: %d", spk, strm)

    -- Frustum culling setting
    local frustum = core.get_setting("graphics.frustum-culling")
    document.dbg_frustum.text = "frustum-culling: "..bool_to_onoff(frustum)

    -- Chunks count and visible
    local chunks = world.count_chunks()
    local visibleChunks = render.get_visible_chunks()
    document.dbg_chunks.text = string.format("chunks: %d visible: %d", chunks, visibleChunks)

    -- Players
    local playersCount = 0
    for _ in pairs(player.get_all()) do playersCount = playersCount + 1 end
    local pid = hud.get_player() or 0
    document.dbg_players.text = string.format("players: %d local: %d", playersCount, pid)

    -- Entities (count and next id)
    local entitiesCount = 0
    for _ in pairs(entities.get_all()) do entitiesCount = entitiesCount + 1 end
    local nextId = render.get_entities_next_id()
    document.dbg_entities.text = string.format("entities: %d next: %d", entitiesCount, nextId)

    -- Seed
    local seed = world.get_seed()
    document.dbg_seed.text = "seed: "..tostring(seed)

    -- Time of day (HH:MM)
    local t = world.get_day_time()
    local timeSeconds = math.floor(t * 24 * 60 * 60)
    local timeFormatted = string.formatted_time(timeSeconds)
    local hoursString = string.left_pad(tostring(timeFormatted.h), 2, '0')
    local minutesString = string.left_pad(tostring(timeFormatted.m), 2, '0')
    document.dbg_time_label.text = string.format("%s:%s", hoursString, minutesString)

    -- Mesh/Draw-calls and Particles
    document.dbg_meshes.text = string.format("meshes: %d", render.get_meshes_count())
    document.dbg_draw_calls.text = string.format("draw-calls: %d", render.get_draw_calls())
    local p = render.get_particles_visible()
    local e = render.get_emitters_alive()
    document.dbg_particles.text = string.format("particles: %d emitters: %d", p, e)

    -- Selection details and block state
    local selx, sely, selz = player.get_selected_block(pid)
    local sid
    if selx ~= nil then
        sid = block.get(selx, sely, selz)
    end
    if selx ~= nil and sid ~= -1 and sid ~= nil then
        document.dbg_selection_pos.text = string.format("x: %d y: %d z: %d", selx, sely, selz)
        local state = block.get_states(selx, sely, selz)
        local rotation, segment, userBits = block.decompose_state(state)
        document.dbg_block_state.text = string.format(
            "block: %d r:%d s:%s u:%s", sid, rotation[1], to_bin(segment, 3), to_bin(userBits, 8)
        )
        document.dbg_block_name.text = "name: "..block.name(sid)
    else
        document.dbg_selection_pos.text = "x: - y: - z: -"
        document.dbg_block_state.text = "block: -"
        document.dbg_block_name.text = "name: -"
    end

    -- Target entity
    local eid = player.get_selected_entity(pid)
    if eid and entities.exists(eid) then
        local def = entities.get_def(eid)
        local name = entities.def_name(def)
        document.dbg_target_entity.text = string.format("entity: %s uid: %d", tostring(name or "-"), eid)
    else
        document.dbg_target_entity.text = "entity: -"
    end
end

local function update_network_speed()
    local totalDownload = network.get_total_download()
    local totalUpload = network.get_total_upload()
    local down = totalDownload - (lastTotalDownload or 0)
    local up = totalUpload - (lastTotalUpload or 0)
    lastTotalDownload = totalDownload
    lastTotalUpload = totalUpload
    netSpeedString = string.format("net: download: %d B/s upload: %d B/s", math.max(down, 0), math.max(up, 0))
    document.dbg_net.text = netSpeedString
end

function on_open()
    if initialized then return end
    initialized = true

    update_debug_info()

    document.root:setInterval(16, function()
        update_fps_minmax()
        update_debug_info()
    end)

    document.root:setInterval(500, function()
        update_fps_text()
    end)

    document.root:setInterval(1000, function()
        update_network_speed()
    end)
end

function on_close()
    initialized = false
    fps, fpsMin, fpsMax = 0, 0, 0
    lastTotalDownload, lastTotalUpload = 0, 0
end
