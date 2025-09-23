local events = {
    handlers = {}
}

function events.on(event, func)
    if events.handlers[event] == nil then
        events.handlers[event] = {}
    end
    table.insert(events.handlers[event], func)
end

function events.reset(event, func)
    if func == nil then
        events.handlers[event] = nil
    else
        events.handlers[event] = {func}
    end
end

function events.remove_by_prefix(prefix)
    for name, handlers in pairs(events.handlers) do
        local actualname = name
        if type(name) == 'table' then
            actualname = name[1]
        end
        if actualname:sub(1, #prefix+1) == prefix..':' then
            events.handlers[actualname] = nil
        end
    end
end

function events.emit(event, ...)
    local result = nil
    local handlers = events.handlers[event]
    if handlers == nil then
        return nil
    end
    for _, func in ipairs(handlers) do
        local status, newres = xpcall(func, __vc__error, ...)
        if not status then
            debug.error("error in event ("..event..") handler: "..newres)
        else 
            result = result or newres
        end
    end
    return result
end
return events
