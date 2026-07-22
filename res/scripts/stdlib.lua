local __app = __vc_app
local enable_experimental = __app.get_setting("debug.enable-experimental")

------------------------------------------------
------ Extended kit of standard functions ------
------------------------------------------------

function sleep(timesec)
    local start = time.uptime()
    while time.uptime() - start < timesec do
        coroutine.yield()
    end
end

local function tb_frame_tostring(frame)
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

local __vc__app_script_coroutine

local function complete_app_lib(app)
    local __app_load_content = app.load_content
    local __app_reset_content = app.reset_content
    local __app_reconfig_packs = app.reconfig_packs
    local __app_tick = coroutine.yield
    local __app_set_setting = app.set_setting
    local __app_quit = app.quit

    app.sleep = sleep
    app.script = __VC_SCRIPT_NAME

    app.set_setting = function(name, value, ...)
        __app_set_setting(name, value, ...)
        events.emit("core:setting."..name..".set", value)
    end
    app.tick = __app_tick

    local function call_in_app_script_co(func, ...)
        if __vc_is_post_runnable_context() then
            func(...)
            return
        end
        local running = coroutine.running()
        if not running or running ~= __vc__app_script_coroutine then
            error("content must be reload in application script coroutine")
        end
        func(...)
        __app_tick()
    end

    app.reconfig_packs = function(...)
        call_in_app_script_co(__app_reconfig_packs, ...)
    end
    app.load_content = function(...)
        call_in_app_script_co(__app_load_content, ...)
    end
    app.reset_content = function(...)
        call_in_app_script_co(__app_reset_content, ...)
    end

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

    function app.quit(silent)
        if not silent then
            local tb = debug.get_traceback(1)
            local s = "app.quit() traceback:"
            for i, frame in ipairs(tb) do
                s = s .. "\n\t"..tb_frame_tostring(frame)
            end
            debug.log(s)
        end
        __app_quit()
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

complete_app_lib(__app)
require "core:internal/maths_inline"
require "core:internal/debugging"
require "core:internal/audio_input"
require "core:internal/extensions/inventory"
asserts = require "core:internal/asserts"
events = require "core:internal/events"

function pack.unload(prefix)
    events.remove_by_prefix(prefix)
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
            if vc.is_headless() then
                return fullmsg
            end
            gui.alert(fullmsg, function()
                if world.is_open() then
                    __app.close_world()
                else
                    __app.reset_content()
                    menu:reset()
                    menu.page = "main"
                end
            end)
            return fullmsg
        end)
        if not status then
            debug.error(error)
        end
    end)
    __vc_named_coroutines[name] = co
end

function __vc_update_coroutines()
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
end

function __vc_start_app_script(path, name)
    debug.log("starting application script "..path)

    local code = file.read(path)
    local chunk, err = loadstring(code, path)
    if chunk == nil then
        error(err)
    end
    local script_env = setmetatable({app = __app}, {__index=_G})
    chunk = setfenv(chunk, script_env)
    if name then
        start_coroutine(chunk, name)
        __vc__app_script_coroutine = __vc_named_coroutines[name]
        return
    else
        local id = __vc_start_coroutine(chunk)
        __vc__app_script_coroutine = __vc_coroutines[id]
        return id
    end
end

gui_util = require "core:internal/gui_util"

Document = gui_util.Document
Element = gui_util.Element
RadioGroup = gui_util.RadioGroup
__vc_page_loader = function(...) return gui_util.load_page(__app, ...) end

function __vc_get_document_node(docname, nodeid)
    return Element.new(docname, nodeid)
end

_GUI_ROOT = Document.new("core:root")
_MENU = _GUI_ROOT.menu
menu = _MENU
gui.root = _GUI_ROOT
gui.main_frame_id = "core:main"

function gui.close_menu()
    if menu then
        menu:reset()
    end
    gui.set_active_frame("")
end

local __gui_create_frame = gui.create_frame
function gui.create_frame(id, output, size)
    __gui_create_frame(id, output, size)

    local document = Document.new(id)
    return document.root, document
end

do
    local status, err = pcall(function()
        local default_styles = toml.parse(file.read(
            "res:devtools/default_syntax_scheme.toml"
        ))
        gui.set_syntax_styles(default_styles)
    end)
    if not status then
        debug.error("could not to load default syntax scheme: "..err)
    end
end

function gui.process_template(source, params)
    local text = source:gsub("%%{([^}]+)}", function(n)
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

function gui.template(name, params)
    local text = file.read(file.find("layouts/templates/"..name..".xml"))
    return gui.process_template(text, params)
end

session = require "core:internal/session"
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
world.raycast = entities.__world_raycast
entities.__world_raycast = nil

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
time.schedules = {}

table.merge(_G, require "core:internal/lifetime_events")

core.get_core_token = audio.input.__get_core_token

require "core:internal/console"
require "core:internal/deprecated"


------------------------------------------
---- Unsafe functions cleanup section ----
------------------------------------------

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

ffi = nil
__vc_app = nil
__vc_lock_internal_modules()
__vc_lock_internal_modules = nil
__vc_update_coroutines = nil
__VC_SCRIPT_NAME = ""
