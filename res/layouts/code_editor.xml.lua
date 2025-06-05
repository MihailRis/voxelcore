local writeables = {}
local registry

local current_file = {
    filename = "",
    mutable = nil
}

local warnings_all = {}
local errors_all = {}

local warning_id = 0
local error_id = 0

events.on("core:warning", function (wtype, text, traceback)
    local full = wtype..": "..text
    if table.has(warnings_all, full) then
        return
    end
    local encoded = base64.encode(bjson.tobytes({frames=traceback}))
    document.problemsLog:add(gui.template("problem", {
        type="warning", 
        text=full, 
        traceback=encoded, 
        id=tostring(warning_id)
    }))
    warning_id = warning_id + 1
    table.insert(warnings_all, full)
end)

events.on("core:error", function (msg, traceback)
    local _, endindex = string.find(msg, ": ")
    local full = ""
    for i,frame in ipairs(traceback) do
        full = full..frame.source..tostring(frame.currentline)
    end
    if table.has(errors_all, full) then
        return
    end
    local encoded = base64.encode(bjson.tobytes({frames=traceback}))
    document.problemsLog:add(gui.template("problem", {
        type="error", 
        text=msg:sub(endindex), 
        traceback=encoded,
        id=tostring(error_id)
    }))
    error_id = error_id + 1
    table.insert(errors_all, full)
end)

events.on("core:open_in_editor", function(filename, linenum)
    open_file_in_editor(filename, linenum)
end)

local function find_mutable(filename)
    local packid = file.prefix(filename)
    if packid == "core" then
        return
    end
    local saved = writeables[packid]
    if saved then
        return saved..":"..file.path(filename)
    end
    local packinfo = pack.get_info(packid)
    if not packinfo then
        return
    end
    local path = packinfo.path
    if file.is_writeable(path) then
        return file.join(path, file.path(filename))
    end
end

local function refresh_file_title()
    if current_file.filename == "" then
        document.title.text = ""
        return
    end
    local edited = document.editor.edited
    current_file.modified = edited
    document.saveIcon.enabled = edited
    document.title.text = gui.str('File')..' - '..current_file.filename
        ..(edited and ' *' or '')
end

function on_control_combination(keycode)
    if keycode == input.keycode("s") then
        save_current_file()
    elseif keycode == input.keycode("r") then
        run_current_file()
    end
end

function unlock_access()
    if current_file.filename == "" then
        return
    end
    pack.request_writeable(file.prefix(current_file.filename), 
        function(token)
            writeables[file.prefix(current_file.filename)] = token
            current_file.mutable = token..":"..file.path(current_file.filename)
            open_file_in_editor(current_file.filename, 0, current_file.mutable)
        end
    )
end

local function reload_model(filename, name)
    assets.parse_model("xml", document.editor.text, name)
end

function run_current_file()
    if not current_file.filename then
        return
    end

    local info = registry.get_info(current_file.filename)
    local script_type = info and info.type or "file"
    local unit = info and info.unit

    if script_type == "model" then
        print(current_file.filename)
        clear_output()
        local _, err = pcall(reload_model, current_file.filename, unit)
        if err then
            document.output:paste(string.format("\n[#FF0000]%s[#FFFFFF]", err))
        end
        return
    end

    local chunk, err = loadstring(document.editor.text, current_file.filename)
    clear_output()
    if not chunk then
        local line, message = err:match(".*:(%d*): (.*)")
        document.output:paste(
            string.format(
                "\n[#FF3030]%s: %s[#FFFFFF]", 
                gui.str("Error at line %{0}"):gsub("%%{0}", line), message)
        )
        return
    end

    save_current_file()

    local func = function()
        local stack_size = debug.count_frames()
        xpcall(chunk, function(msg) __vc__error(msg, 1, 1, stack_size) end)
    end

    local funcs = {
        block = block.reload_script,
        item = item.reload_script,
        world = world.reload_script,
        hud = hud.reload_script,
        component = entities.reload_component,
        module = reload_module,
    }
    func = funcs[script_type] or func
    local output = core.capture_output(function() func(unit) end)
    document.output:paste(string.format("\n%s", output))
end

function clear_traceback()
    local tb_list = document.traceback
    tb_list:clear()
    tb_list:add("<label enabled='false' margin='2'>@devtools.traceback</label>")
end

function clear_output()
    local output = document.output
    output.text = ""
    output:paste("[#FFFFFF80]"..gui.str("devtools.output").."[#FFFFFF]")
end

events.on("core:open_traceback", function(traceback_b64)
    local traceback = bjson.frombytes(base64.decode(traceback_b64))

    clear_traceback()

    local tb_list = document.traceback
    local srcsize = tb_list.size
    for _, frame in ipairs(traceback.frames) do
        local callback = ""
        local framestr = ""
        if frame.what == "C" then
            framestr = "C/C++ "
        else
            framestr = frame.source..":"..tostring(frame.currentline).." "
            if file.exists(frame.source) then
                callback = string.format(
                    "open_file_in_editor('%s', %s)",
                    frame.source, frame.currentline-1
                )
            else
                callback = "document.editor.text = 'Could not open source file'"
            end
        end
        if frame.name then
            framestr = framestr.."("..tostring(frame.name)..")"
        end
        local color = "#FFFFFF"
        tb_list:add(gui.template("stack_frame", {
            location=framestr, 
            color=color,
            callback=callback,
            enabled=file.exists(frame.source)
        }))
    end
    tb_list.size = srcsize
end)

--- Save the current file in the code editor if has writeable path.
function save_current_file()
    if not current_file.mutable then
        return
    end
    file.write(current_file.mutable, document.editor.text)
    current_file.modified = false
    document.saveIcon.enabled = false
    document.title.text = gui.str('File')..' - '..current_file.filename
    document.editor.edited = false
end

--- Open a file in the code editor.
--- @param filename string - the path to the file to open.
--- @param line integer - the line number to focus on (optional).
--- @param mutable string - writeable file path (optional).
function open_file_in_editor(filename, line, mutable)
    debug.log("opening file " .. string.escape(filename) .. " in editor")

    local ext = file.ext(filename)
    if ext == "xml" or ext == "vcm" then
        document.modelviewer.src = file.stem(filename)
        document.modelviewer.visible = true
    else
        document.modelviewer.visible = false
    end
    document.codePanel:refresh()

    local editor = document.editor
    local source = file.read(filename):gsub('\t', '    ')
    editor.scroll = 0
    editor.text = source
    editor.focused = true
    editor.syntax = file.ext(filename)
    if line then
        time.post_runnable(function()
            editor.caret = editor:linePos(line)
        end)
    end
    document.title.text = gui.str('File') .. ' - ' .. filename
    current_file.filename = filename
    current_file.mutable = mutable or find_mutable(filename)
    document.lockIcon.visible = current_file.mutable == nil
    document.editor.editable = current_file.mutable ~= nil
    document.saveIcon.enabled = current_file.modified
end

function on_open(mode)
    registry = require "core:internal/scripts_registry"

    document.codePanel:setInterval(200, refresh_file_title)

    clear_traceback()
    clear_output()
end
