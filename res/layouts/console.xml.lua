console_mode = "console"

history = session.get_entry("commands_history")
history_pointer = #history

events.on("core:open_traceback", function()
    if modes and modes.current ~= 'debug' then
        modes:set('debug')
    end
end)

function setup_variables()
    local pid = hud.get_player()
    local x,y,z = player.get_pos(pid)
    console.set("player", pid)
    console.set('pos.x', x)
    console.set('pos.y', y)
    console.set('pos.z', z)
    local pentity = player.get_entity(pid)
    if pentity > 0 then
        console.set('entity.id', pentity)
    end
    local sentity = player.get_selected_entity(pid)
    if sentity ~= nil then
        console.set('entity.selected', sentity)
    end
end

function on_history_up()
    if history_pointer == 0 then
        return
    end
    document.prompt.text = history[history_pointer]
    document.prompt.caret = -1
    history_pointer = history_pointer - 1
end

function on_history_down()
    if history_pointer >= #history-1 then
        return
    end
    history_pointer = history_pointer + 1
    document.prompt.text = history[history_pointer + 1]
    document.prompt.caret = -1
end

function add_to_history(text)
    table.insert(history, text)
    history_pointer = #history
end

function submit(text)
    if #text == 0 then
        document.prompt.focused = true
        return
    end
    text = text:trim()
    add_to_history(text)

    if console_mode == "chat" then
        if not text:starts_with("/") then
            text = "chat "..string.escape(text)
        else
            text = text:sub(2)
        end
    end
    
    local name
    for s in text:gmatch("%S+") do
        name = s
        break
    end
    if name == nil then
        name = text
    end
    if not rules.get("allow-cheats") and table.has(console.cheats, name) then
        console.log("cheat commands are disabled")
        document.prompt.text = ""
        document.prompt.focused = true
        return
    end
    
    document.log.caret = -1
    document.prompt.text = ""
    document.prompt.focused = true

    setup_variables()

    if console.submit then
        console.submit(text)
    else    
        local status, result = pcall(console.execute, text)
        if result then
            console.log(result)
        end
    end
end

function set_mode(mode)
    local show_prompt = mode == 'chat' or mode == 'console'

    document.editorRoot.visible = mode == 'debug'
    document.logContainer.visible = mode ~= 'debug'

    if mode == 'debug' then
        document.root.color = {16, 18, 20, 220}
        document.editorRoot.src = "core:code_editor"
    else
        document.root.color = {0, 0, 0, 128}
        document.editorRoot.src = ""
    end

    document.prompt.visible = show_prompt
    if show_prompt then
        document.prompt.focused = true
    end
    console_mode = mode
end

function on_open(mode)
    if modes == nil then
        modes = RadioGroup({
            chat=document.s_chat,
            console=document.s_console,
            debug=document.s_debug
        }, function (mode)
            set_mode(mode)
        end, mode or "console")
    elseif mode then
        modes:set(mode)
    end
    hud.close("core:ingame_chat")
end

function on_close()
    time.post_runnable(function()
        hud.open_permanent("core:ingame_chat")
    end)
end
