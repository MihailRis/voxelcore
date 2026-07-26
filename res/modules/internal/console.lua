
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
