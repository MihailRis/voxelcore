local breakpoints = {}
local dbg_steps_mode = false
local dbg_step_into_func = false
local hook_lock = false
local current_func
local current_func_stack_size

local __parse_path = parse_path
local _debug_getinfo = debug.getinfo
local _debug_getlocal = debug.getlocal
local __pause = debug.pause
local __error = error
local __sethook = debug.sethook

-- 'return' hook not called for some functions
-- todo: speedup
local function calc_stack_size()
    local s = debug.traceback("", 2)
    local count = 0
    for i in s:gmatch("\n") do
        count = count + 1
    end
    return count
end

local is_debugging = debug.is_debugging()
if is_debugging then
    __sethook(function (e, line)
        if e == "return" then
            local info = _debug_getinfo(2)
            if info.func == current_func then
                current_func = nil
            end
        end
        if dbg_steps_mode and not hook_lock then
            hook_lock = true

            if not dbg_step_into_func then
                local func = _debug_getinfo(2).func
                if func ~= current_func then
                    return
                end
                if current_func_stack_size ~= calc_stack_size() then
                    return
                end
            end
            current_func = func
            __pause("step")
            debug.pull_events()
        end
        hook_lock = false
        local bps = breakpoints[line]
        if not bps then
            return
        end
        local source = _debug_getinfo(2).source
        if not bps[source] then
            return
        end
        current_func = _debug_getinfo(2).func
        current_func_stack_size = calc_stack_size()
        __pause("breakpoint")
        while debug.pull_events() do
            __pause()
        end
    end, "lr")
end

local DBG_EVENT_SET_BREAKPOINT = 1
local DBG_EVENT_RM_BREAKPOINT = 2
local DBG_EVENT_STEP = 3
local DBG_EVENT_STEP_INTO_FUNCTION = 4
local DBG_EVENT_RESUME = 5
local DBG_EVENT_GET_VALUE = 6
local __pull_events = debug.__pull_events
local __sendvalue = debug.__sendvalue
debug.__pull_events = nil
debug.__sendvalue = nil

function debug.get_pack_by_frame(func)
    local prefix, _ = __parse_path(_debug_getinfo(func, "S").source)
    return prefix
end

function debug.pull_events()
    if not is_debugging then
        return
    end
    if not debug.is_debugging() then
        is_debugging = false
        __sethook()
    end
    local events = __pull_events()
    if not events then
        return
    end
    local keepPaused = false
    for i, event in ipairs(events) do
        if event[1] == DBG_EVENT_SET_BREAKPOINT then
            debug.set_breakpoint(event[2], event[3])
        elseif event[1] == DBG_EVENT_RM_BREAKPOINT then
            debug.remove_breakpoint(event[2], event[3])
        elseif event[1] == DBG_EVENT_STEP then
            dbg_steps_mode = true
            dbg_step_into_func = false
        elseif event[1] == DBG_EVENT_STEP_INTO_FUNCTION then
            dbg_steps_mode = true
            dbg_step_into_func = true
        elseif event[1] == DBG_EVENT_RESUME then
            dbg_steps_mode = false
            dbg_step_into_func = false
        elseif event[1] == DBG_EVENT_GET_VALUE then
            local _, value = _debug_getlocal(event[2] + 3, event[3])
            for _, key in ipairs(event[4]) do
                if value == nil then
                    value = "error: index nil value"
                    break
                end
                value = value[key]
            end
            __sendvalue(value, event[2], event[3], event[4])
            keepPaused = true
        end
    end
    return keepPaused
end

function debug.set_breakpoint(source, line)
    local bps = breakpoints[line]
    if not bps then
        bps = {}
        breakpoints[line] = bps
    end
    bps[source] = true
end

function debug.remove_breakpoint(source, line)
    local bps = breakpoints[line]
    if not bps then
        return
    end
    bps[source] = nil
end

function error(message, level)
    if is_debugging then
        __pause("exception", message)
    end
    __error(message, level)
end
