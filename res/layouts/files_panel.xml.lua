local registry
local filenames

function filter_files(text)
    local pattern_safe = text:pattern_safe();
    local filtered = {}
    for _, filename in ipairs(filenames) do
        if filename:find(pattern_safe) then
            table.insert(filtered, filename)
        end
    end
    build_files_list(filtered, pattern_safe)
end

function open_file_in_editor(filename, linenum)
    events.emit("core:open_in_editor", filename, linenum)
end

function build_files_list(filenames, highlighted_part)
    local files_list = document.filesList
    files_list.scroll = 0
    files_list:clear()

    for _, actual_filename in ipairs(filenames) do
        local filename = actual_filename
        if highlighted_part then
            filename = filename:gsub(highlighted_part, "**"..highlighted_part.."**")
        end
        local parent = file.parent(filename)
        local info = registry.get_info(actual_filename)
        local icon = "file"
        if info then
            icon = info.type == "component" and "entity" or info.type
        end
        files_list:add(gui.template("script_file", {
            path = parent .. (parent[#parent] == ':' and '' or '/'), 
            name = file.name(filename),
            icon = icon,
            unit = info and info.unit or '',
            filename = actual_filename,
            open_func = "open_file_in_editor",
        }))
    end
end

function on_open(mode)
    registry = require "core:internal/scripts_registry"
    
    local files_list = document.filesList

    filenames = registry.filenames
    table.sort(filenames)
    build_files_list(filenames)
end
