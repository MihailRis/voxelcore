local __app = __vc_app
local __rules = rules

local function configure_SSAO()
    -- Temporary using slot to configure built-in SSAO effect
    local slot = gfx.posteffects.index("core:ssao")
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

    local function update_ssao_quality(value)
        value = math.min(value, 3)
        gfx.posteffects.set_params(slot, {
            u_kernelSize = value * 16,
            u_radius = 0.4 / value,
            u_bias = 0.006 / value / value,
        })
    end
    events.on("core:setting.graphics.ssao.set", update_ssao_quality)

    update_ssao_quality(__app.get_setting("graphics.ssao"))
end

local function __vc_on_hud_open()
    local _hud_is_content_access = hud._is_content_access
    local _hud_set_content_access = hud._set_content_access
    local _hud_set_debug_cheats = hud._set_debug_cheats

    __rules.create("allow-cheats", true)

    __rules.create("allow-content-access", _hud_is_content_access(), function(value)
        _hud_set_content_access(value)
    end)
    __rules.create("allow-flight", true, function(value)
        input.set_enabled("player.flight", value)
    end)
    __rules.create("allow-noclip", true, function(value)
        input.set_enabled("player.noclip", value)
    end)
    __rules.create("allow-attack", true, function(value)
        input.set_enabled("player.attack", value)
    end)
    __rules.create("allow-destroy", true, function(value)
        input.set_enabled("player.destroy", value)
    end)
    __rules.create("allow-cheat-movement", true, function(value)
        input.set_enabled("movement.cheat", value)
    end)
    __rules.create("allow-fast-interaction", true, function(value)
        input.set_enabled("player.fast_interaction", value)
    end)
    __rules.create("allow-debug-cheats", true, function(value)
        _hud_set_debug_cheats(value)
    end)
    input.add_callback("devtools.console", function()
        if menu.page ~= "" then
            return
        end
        time.post_runnable(function()
            hud.show_overlay("core:console", false, {"console"})
        end)
    end)
    input.add_callback("hud.chat", function()
        if menu.page ~= "" then
            return
        end
        time.post_runnable(function()
            hud.show_overlay("core:console", false, {"chat"})
        end)
    end)
    input.add_callback("key:escape", function()
        if menu.page ~= "" then
            if not menu:back() then
                menu:reset()
                gui.set_active_frame("")
            end
        elseif hud.is_inventory_open() then
            hud.close_inventory()
        elseif gui.get_active_frame() then
            gui.set_active_frame("")
        else
            hud.pause()
        end
    end)
    hud.open_permanent("core:ingame_chat")

    configure_SSAO()
end

local Schedule = require "core:schedule"
local ScheduleGroup_mt = {
    __index = {
        publish = function(self, schedule)
            local id = self._next_schedule
            self._schedules[id] = schedule
            self._next_schedule = id + 1
        end,
        tick = function(self, dt)
            for id, schedule in pairs(self._schedules) do
                schedule:tick(dt)
            end
            self.common:tick(dt)
        end,
        remove = function(self, id)
            self._schedules[id] = nil
        end,
    }
}

local function ScheduleGroup()
    return setmetatable({
        _next_schedule = 1,
        _schedules = {},
        common = Schedule()
    }, ScheduleGroup_mt)
end

local RULES_FILE = "world:rules.toml"
local function __vc_on_world_open()
    time.schedules.world = ScheduleGroup()

    if not file.exists(RULES_FILE) then
        return
    end
    local rule_values = toml.parse(file.read(RULES_FILE))
    for name, value in pairs(rule_values) do
        __rules.set(name, value)
    end
end

local function __vc_on_world_tick(tps)
    time.schedules.world:tick(1.0 / tps)
end

local function __vc_process_before_quit()
    block.__process_register_events()
end

local function __vc_on_world_save()
    local rule_values = {}
    for name, rule in pairs(__rules.rules) do
        rule_values[name] = rule.value
    end
    file.write(RULES_FILE, toml.tostring(rule_values))
end

local __close_all_descriptors = file.__close_all_descriptors
local __gui_util_reset_local = gui_util.__reset_local
local __stdcomp_reset = stdcomp.__reset
file.__close_all_descriptors = nil
gui_util.__reset_local = nil
stdcomp.reset = nil

local function __vc_on_world_quit()
    __rules.clear()
    __gui_util_reset_local()
    __stdcomp_reset()
    __close_all_descriptors()
end

local __post_runnables = {}
local fn_audio_reset_fetch_buffer = audio.__reset_fetch_buffer
audio.__reset_fetch_buffer = nil

local __vc_update_coroutines = _G.__vc_update_coroutines

local function __process_post_runnables()
    if #__post_runnables > 0 then
        for _, func in ipairs(__post_runnables) do
            local status, result = xpcall(func, __vc__error)
            if not status then
                debug.error("error in post_runnable: "..result)
            end
        end
        __post_runnables = {}
    end

    __vc_update_coroutines()

    fn_audio_reset_fetch_buffer()
    debug.pull_events()
    network.__process_events()
    if not hud or not hud.is_paused() then
        block.__process_register_events()
        block.__perform_ticks(time.delta())
    end
end

local __vc__is_post_runnable = false

function __vc__process_post_runnables()
    __vc__is_post_runnable = true
    local success, err = pcall(__process_post_runnables)
    if not success then
        debug.error("an error ocurred while processing post-runnables: ".. err)
    end
    __vc__is_post_runnable = false
end

function time.post_runnable(runnable)
    table.insert(__post_runnables, runnable)
end

return {
    __vc_on_hud_open = __vc_on_hud_open,
    __vc_on_world_open = __vc_on_world_open,
    __vc_on_world_tick = __vc_on_world_tick,
    __vc_process_before_quit = __vc_process_before_quit,
    __vc_on_world_save = __vc_on_world_save,
    __vc_on_world_quit = __vc_on_world_quit,
    __vc__process_post_runnables = __vc__process_post_runnables,
    __vc_is_post_runnable_context = function()
        return __vc__is_post_runnable
    end
}
