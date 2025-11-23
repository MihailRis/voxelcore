local enable_experimental = core.get_setting("debug.enable-experimental")

------------------------------------------------
------ Extended kit of standard functions ------
------------------------------------------------

function sleep(timesec)
    local start = time.uptime()
    while time.uptime() - start < timesec do
        coroutine.yield()
    end
end

function tb_frame_tostring(frame)
    local s = frame.short_src
    if frame.what ~= "C" then
        s = s .. ":" .. tostring(frame.currentline)
    end
    if frame.what == "main" then
        s = s .. ": in main chunk"
    elseif frame.name then
        s = s .. ": in function " .. utf8.escape(frame.name)
    end
    return s
end

local function complete_app_lib(app)
    app.sleep = sleep
    app.script = __VC_SCRIPT_NAME
    app.new_world = core.new_world
    app.open_world = core.open_world
    app.save_world = core.save_world
    app.close_world = core.close_world
    app.reopen_world = core.reopen_world
    app.delete_world = core.delete_world
    app.reconfig_packs = core.reconfig_packs
    app.get_setting = core.get_setting
    app.set_setting = core.set_setting
    app.tick = function()
        coroutine.yield()
    end
    app.get_version = core.get_version
    app.get_setting_info = core.get_setting_info
    app.load_content = function()
        core.load_content()
        app.tick()
    end
    app.reset_content = core.reset_content
    app.is_content_loaded = core.is_content_loaded
    app.set_title = core.set_title
    
    function app.config_packs(packs_list)
        -- Check if packs are valid and add dependencies to the configuration
        packs_list = pack.assemble(packs_list)
        
        local installed = pack.get_installed()
        local toremove = {}
        for _, packid in ipairs(installed) do
            if not table.has(packs_list, packid) then
                table.insert(toremove, packid)
            end
        end
        local toadd = {}
        for _, packid in ipairs(packs_list) do
            if not table.has(installed, packid) then
                table.insert(toadd, packid)
            end
        end
        app.reconfig_packs(toadd, toremove)
    end

    function app.quit()
        local tb = debug.get_traceback(1)
        local s = "app.quit() traceback:"
        for i, frame in ipairs(tb) do
            s = s .. "\n\t"..tb_frame_tostring(frame)
        end
        debug.log(s)
        core.quit()
        coroutine.yield()
    end

    function app.sleep_until(predicate, max_ticks, max_time)
        max_ticks = max_ticks or 1e9
        max_time = max_time or 1e9
        local ticks = 0
        local start_time = os.clock()
        while ticks < max_ticks and
            os.clock() - start_time < max_time
            and not predicate() do
            app.tick()
            ticks = ticks + 1
        end
        if os.clock() - start_time >= max_time then
            error("timeout")
        end
        if ticks == max_ticks then
            error("max ticks exceed")
        end
    end
end

if app then
    complete_app_lib(app)
elseif __vc_app then
    complete_app_lib(__vc_app)
end

require "core:internal/maths_inline"
require "core:internal/debugging"
require "core:internal/audio_input"
require "core:internal/extensions/inventory"
asserts = require "core:internal/asserts"
events = require "core:internal/events"

function pack.unload(prefix)
    events.remove_by_prefix(prefix)
end

function __vc_start_app_script(path, name)
    debug.log("starting application script "..path)

    local code = file.read(path)
    local chunk, err = loadstring(code, path)
    if chunk == nil then
        error(err)
    end
    local script_env = setmetatable({app = app or __vc_app}, {__index=_G})
    chunk = setfenv(chunk, script_env)
    if name then
        return start_coroutine(chunk, name)
    else
        return __vc_start_coroutine(chunk)
    end
end

gui_util = require "core:internal/gui_util"

Document = gui_util.Document
Element = gui_util.Element
RadioGroup = gui_util.RadioGroup
__vc_page_loader = gui_util.load_page

function __vc_get_document_node(docname, nodeid)
    return Element.new(docname, nodeid)
end

_GUI_ROOT = Document.new("core:root")
_MENU = _GUI_ROOT.menu
menu = _MENU
gui.root = _GUI_ROOT

---  Console library extension ---
console.cheats = {}

local log_element = Document.new("core:console").log
function console.log(...)
    local args = {...}
    local text = ''
    for i,v in ipairs(args) do
        if i ~= 1 then 
            text = text..' '..v 
        else
            text = text..v
        end
    end
    log_element.caret = -1
    if log_element.caret > 0 then
        text = '\n'..text
    end
    log_element:paste(text)
end

local console_add_command = console.__add_command
console.__add_command = nil

function console.add_command(scheme, description, handler, is_cheat)
    console_add_command(scheme, description, handler)
    if not is_cheat then return end

    local name = string.match(scheme, "^(%S+)")
    if not name then
        error("Incorrect command syntax, command name not found")
    end

    table.insert_unique(console.cheats, name)
end

function console.is_cheat(name)
    if not table.has(console.get_commands_list(), name) then
        error(string.format("command \"%s\" not found", name))
    end

    return table.has(console.cheats, name)
end

function console.set_cheat(name, status)
    local is_cheat = console.is_cheat(name)
    if status and not is_cheat then
        table.insert(console.cheats, name)
        return true
    elseif not status and is_cheat then
        table.remove_value(console.cheats, name)
        return true
    end

    return false
end

function console.chat(...)
    console.log(...)
    events.emit("core:chat", ...)
end

function gui.template(name, params)
    local text = file.read(file.find("layouts/templates/"..name..".xml"))
    text = text:gsub("%%{([^}]+)}", function(n) 
        local s = params[n]
        if s == nil then
            return
        end
        if type(s) ~= "string" then
            return tostring(s)
        end
        if #s == 0 then
            return ''
        end
        local e = string.escape(s)
        return e:sub(2, #e-1)
    end)
    text = text:gsub('if%s*=%s*[\'"]%%{%w+}[\'"]', "if=\"\"")
    -- remove unsolved properties: attr='%{var}'
    text = text:gsub('%s*%S+=[\'"]%%{[^}]+}[\'"]%s*', " ")
    return text
end

session = {
    entries={}
}

function session.get_entry(name)
    local entry = session.entries[name]
    if entry == nil then
        entry = {}
        session.entries[name] = entry
    end
    return entry
end

function session.reset_entry(name)
    session.entries[name] = nil
end

stdcomp = require "core:internal/stdcomp"
entities.get = stdcomp.get_Entity
entities.get_all = function(uids)
    if uids == nil then
        local values = {}
        for k,v in pairs(stdcomp.get_all()) do
            values[k] = v
        end
        return values 
    else
        return stdcomp.get_all(uids)
    end
end

__vc_scripts_registry = require "core:internal/scripts_registry"

file.open = require "core:internal/stream_providers/file"
file.open_named_pipe = require "core:internal/stream_providers/named_pipe"

if ffi.os == "Windows" then
    ffi.cdef[[
    unsigned long GetCurrentProcessId();
    ]]
    
    os.pid = ffi.C.GetCurrentProcessId()
else
    ffi.cdef[[
    int getpid(void);
    ]]

    os.pid = ffi.C.getpid()
end

math.randomseed(time.uptime() * 1536227939)

rules = require "core:internal/rules"
local _rules = rules

function __vc_on_hud_open()
    local _hud_is_content_access = hud._is_content_access
    local _hud_set_content_access = hud._set_content_access
    local _hud_set_debug_cheats = hud._set_debug_cheats

    _rules.create("allow-cheats", true)

    _rules.create("allow-content-access", _hud_is_content_access(), function(value)
        _hud_set_content_access(value)
    end)
    _rules.create("allow-flight", true, function(value)
        input.set_enabled("player.flight", value)
    end)
    _rules.create("allow-noclip", true, function(value)
        input.set_enabled("player.noclip", value)
    end)
    _rules.create("allow-attack", true, function(value)
        input.set_enabled("player.attack", value)
    end)
    _rules.create("allow-destroy", true, function(value)
        input.set_enabled("player.destroy", value)
    end)
    _rules.create("allow-cheat-movement", true, function(value)
        input.set_enabled("movement.cheat", value)
    end)
    _rules.create("allow-fast-interaction", true, function(value)
        input.set_enabled("player.fast_interaction", value)
    end)
    _rules.create("allow-debug-cheats", true, function(value)
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
            end
        elseif hud.is_inventory_open() then
            hud.close_inventory()
        else
            hud.pause()
        end
    end)
    hud.open_permanent("core:ingame_chat")
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

time.schedules = {}

local RULES_FILE = "world:rules.toml"
function __vc_on_world_open()
    time.schedules.world = ScheduleGroup()

    if not file.exists(RULES_FILE) then
        return
    end
    local rule_values = toml.parse(file.read(RULES_FILE))
    for name, value in pairs(rule_values) do
        _rules.set(name, value)
    end
end

function __vc_on_world_tick(tps)
    time.schedules.world:tick(1.0 / tps)
end

function __vc_process_before_quit()
    block.__process_register_events()
end

function __vc_on_world_save()
    local rule_values = {}
    for name, rule in pairs(rules.rules) do
        rule_values[name] = rule.value
    end
    file.write(RULES_FILE, toml.tostring(rule_values))
end

function __vc_on_world_quit()
    _rules.clear()
    gui_util:__reset_local()
    stdcomp.__reset()
    file.__close_all_descriptors()
end

local __vc_coroutines = {}
local __vc_named_coroutines = {}
local __vc_next_coroutine = 1

function __vc_start_coroutine(chunk)
    local co = coroutine.create(chunk)
    local id = __vc_next_coroutine
    __vc_next_coroutine = __vc_next_coroutine + 1
    __vc_coroutines[id] = co
    return id
end

function __vc_resume_coroutine(id)
    local co = __vc_coroutines[id]
    if co then
        local success, err = coroutine.resume(co)
        if not success then
            debug.error(err)
            error(err)
        end
        return coroutine.status(co) ~= "dead"
    end
    return false
end

function __vc_stop_coroutine(id)
    local co = __vc_coroutines[id]
    if co then
        if coroutine.close then
            coroutine.close(co)
        end
        __vc_coroutines[id] = nil
    end
end

function start_coroutine(chunk, name)
    local co = coroutine.create(function()
        local status, error = xpcall(chunk, function(err)
            local fullmsg = "error: "..string.match(err, ": (.+)").."\n"..debug.traceback()
            
            if hud then
                gui.alert(fullmsg, function()
                    if world.is_open() then
                        __vc_app.close_world()
                    else
                        __vc_app.reset_content()
                        menu:reset()
                        menu.page = "main"
                    end
                end)
            end
            return fullmsg
        end)
        if not status then
            debug.error(error)
        end
    end)
    __vc_named_coroutines[name] = co
end

local __post_runnables = {}

local fn_audio_reset_fetch_buffer = audio.__reset_fetch_buffer
audio.__reset_fetch_buffer = nil
core.get_core_token = audio.input.__get_core_token

function __process_post_runnables()
    if #__post_runnables then
        for _, func in ipairs(__post_runnables) do
            local status, result = xpcall(func, __vc__error)
            if not status then
                debug.error("error in post_runnable: "..result)
            end
        end
        __post_runnables = {}
    end

    local dead = {}
    for name, co in pairs(__vc_named_coroutines) do
        local success, err = coroutine.resume(co)
        if not success then
            debug.error(err)
        end
        if coroutine.status(co) == "dead" then
            table.insert(dead, name)
        end
    end
    for _, name in ipairs(dead) do
        __vc_named_coroutines[name] = nil
    end

    fn_audio_reset_fetch_buffer()
    debug.pull_events()
    network.__process_events()
    block.__process_register_events()
    block.__perform_ticks(time.delta())
end

function time.post_runnable(runnable)
    table.insert(__post_runnables, runnable)
end

-- Hide unsafe debug.* functions
local removed_names = {
    "getregistry", "getupvalue", "setupvalue", "upvalueid", "upvaluejoin",
    "sethook", "gethook", "getinfo"
}
local _getinfo = debug.getinfo
for i, name in ipairs(removed_names) do
    debug[name] = nil
end

debug.getinfo = function(lvl, fields)
    if type(lvl) == "number" then
        lvl = lvl + 1
    end
    local debuginfo = _getinfo(lvl, fields)
    debuginfo.func = nil
    return debuginfo
end

require "core:internal/deprecated"

ffi = nil
__vc_lock_internal_modules()
