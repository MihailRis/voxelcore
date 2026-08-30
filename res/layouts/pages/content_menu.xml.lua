local search_utils = require("search_utils")

local MATCHED_COLOR = "[#4FC3F7]"
local NORMAL_COLOR = "[#FFFFFF]"

local packs_installed = {}
local pack_open = {}
local PARSERS = {
    toml = toml,
    json = json
}

local CONFIG_NAMES = {
    "config"
}

function on_open()
    refresh()
    document.search_textbox.focused = true
end

function place_pack(panel, packinfo, callback)
    if packinfo.error then
        callback = nil
    end
    local id_md = packinfo.id_md or packinfo.id
    if packinfo.has_indices then
        packinfo.id_verbose = id_md.."\\*"
    else
        packinfo.id_verbose = id_md
    end
    packinfo.callback = callback
    panel:add(gui.template("pack", packinfo))
end

function refresh_search()
    local search_text = document.search_textbox.text:lower()
    local visible = 0
    local interval = 4
    local step = -1

    local packs = table.copy(packs_installed)

    local score_non_zero_num = 0
    local smart_score_cache = {}
    local colored_ids = {}
    local colored_titles = {}

    local function smart_fuzzy_score(a)
        if smart_score_cache[a.id] then
            return smart_score_cache[a.id]
        end
        local id_score, colored_id = search_utils.fuzzy_score(a.id, search_text,
                                        MATCHED_COLOR, NORMAL_COLOR)
        local title_score, colored_title = search_utils.fuzzy_score(a.title, search_text,
                                                MATCHED_COLOR, NORMAL_COLOR)
        local res = math.max(id_score, title_score)

        smart_score_cache[a.id] = res
        colored_ids[a.id] = colored_id
        colored_titles[a.id] = colored_title

        if res > 0 then
            score_non_zero_num = score_non_zero_num + 1
        end
        return res
    end

    local function cmp(a, b)
        local score_a = smart_fuzzy_score(a)
        local score_b = smart_fuzzy_score(b)
        if score_a == score_b then
            return a.title > b.title
        else
            return score_a > score_b
        end
    end

    table.sort(packs, cmp)

    local contents = document.contents
    contents:clear()

    for i, v in ipairs(packs) do
        local id = v.id

        local packinfo = table.copy(packs[i])
        if colored_ids[id] then
            packinfo.id_md = colored_ids[id]
        end
        if colored_titles[id] then
            packinfo.title = colored_titles[id]
        end

        place_pack(contents, packinfo, callback)

        local content = document["pack_" .. id]
        local pos = content.pos
        local size = content.size

        if i <= score_non_zero_num then
            content.enabled = true
            content.pos = { pos[1], visible * (size[2] + interval) - step }
            visible = visible + 1
        else
            content.enabled = false
            content.pos = { pos[1], (visible + #packs_installed - i) * (size[2] + interval) - step }
        end
    end
end

local function create_checkbox(id, name, cheaked)
    document.configs:add(string.format(
        "<checkbox id='%s' consumer='function(x) set_value(\"%s\", \"%s\", x) end' checked='%s'>%s</checkbox>",
        id, name, id, cheaked, name:gsub("^%l", string.upper)
    ))
end

local function create_textbox(id, name, text)
    document.configs:add(string.format(
        "<label id='%s'>%s</label>",
        id .. "_label", name:gsub("^%l", string.upper)
    ))
    document.configs:add(string.format(
        "<textbox id='%s' consumer='function(x) set_value(\"%s\", \"%s\", x) end' hint='%s'>%s</textbox>",
        id, name, id, name, text
    ))
end

local function create_trackbar(id, name, val)
    document.configs:add(string.format(
        "<label id='%s'>%s (%s)</label>",
        id .. "_label", name:gsub("^%l", string.upper), val
    ))
    document.configs:add(string.format( 
        "<trackbar id='%s' consumer='function(x) set_value(\"%s\", \"%s\", x) end' value='%s' min='0' max='1000' step='10'>%s</trackbar>",
        id, name, id, val, name
    ))
end

local function create_label(id, text, color)
    if id then
        document.configs:add(string.format(
            "<label markup='md' id='%s' color='%s'>%s</label>",
            id, color, text
        ))
    else
        document.configs:add(string.format(
            "<label markup='md' color='%s'>%s</label>",
            color, text
        ))
    end
end

local function create_config(i, config, name, path)
    create_label(name, ('**[' .. name .. ']**'):upper(), "#FFFFFF")
    pack_open[2][i] = {elements = {}, path = path}
    for _, a in ipairs(config.checkboxes) do
        create_checkbox(i .. '_' .. a[1], a[1], a[2])
        pack_open[2][i]["elements"][a[1]] = type(a[2])
    end
    create_label(nil, '', 0)
    for _, a in ipairs(config.trackbars) do
        create_trackbar(i .. '_' .. a[1], a[1], a[2])
        create_label(nil, '', 0)
        pack_open[2][i]["elements"][a[1]] = type(a[2])
    end

    for _, a in ipairs(config.textboxes) do
        create_textbox(i .. '_' .. a[1], a[1], a[2])
        create_label(nil, '', 0)
        pack_open[2][i]["elements"][a[1]] = type(a[2])
    end
    create_label(nil, '', 0)
end

local function valid_name(file_name)
    for _, name in ipairs(CONFIG_NAMES) do
        if string.find(file_name, name) then
            return true
        end
    end

    return false
end

local function has_valid_config(id)
    if not file.exists("config:" .. id) then
        return false
    end

    local files = file.list("config:" .. id)

    for _, file_name in ipairs(files) do
        local name = file_name:match("([^/]+)%.([^%.]+)$")

        if name and valid_name(name) then
            return true
        end
    end

    return false
end

local function load_config_file(path)

    local extension = path:match("^.+%.(.+)$")
    local name = path:match("([^/]+)%.([^%.]+)$")

    if not extension or file.isdir(path) then
        return
    end

    if PARSERS[extension] and valid_name(name:lower()) then
        local value = file.read(path)
        return PARSERS[extension].parse(value), extension
    end
end

local function load_config(path)
    if file.exists(path) then
        local value = load_config_file(path)
        local config = {checkboxes = {},
                        trackbars = {},
                        textboxes = {}
                        }

        if not value then
            return
        end

        for i, v in pairs(value) do
            if type(v) == "string" then
                table.insert(config.textboxes, {i, v})
            elseif type(v) == "boolean" then
                table.insert(config.checkboxes, {i, v})
            elseif type(v) == "number" then
                table.insert(config.trackbars,  {i, v})
            end
        end

        return config
    end
end

local function valid_configs(path)
    if file.exists(path) and file.isdir(path) then
        for _, c in ipairs(file.list(path)) do
            local extension = c:match("^.+%.(.+)$")
            if PARSERS[extension] then
                return true
            end
        end
    end
    return false
end

function set_value(name, id, value)
    local config_id = tonumber(id:match("^(%w+)_"))
    local elem_id = id:match("^[^_]+_(.+)$")
    local type_val = pack_open[2][config_id]["elements"][elem_id]

    if type_val == 'number' then
        document[id .. '_label'].text = string.format("%s (%s)", elem_id:gsub("^%l", string.upper), value)
    end

    local path = pack_open[2][config_id].path
    local config, extension = load_config_file(path)
    config[name] = value

    if PARSERS[extension] then
        file.write(path, PARSERS[extension].tostring(config))
    end
end

function open_pack(id)
    local packinfo = pack.get_info(id)
    pack_open = {id, {}}

    document.configs.visible = false
    document.content_info.visible = true
    if has_valid_config(id) then document.open_config.enabled = true else document.open_config.enabled = false end

    if packinfo['dependencies'] == nil then document.dependencies.text = 'None' else document.dependencies.text = table.tostring(packinfo['dependencies']) end
    if packinfo['creator'] == '' then document.author.text = 'None' else document.author.text = packinfo['creator'] end
    if packinfo['version'] == nil then document.version.text = 'None' else document.version.text = packinfo['version'] end
    if packinfo['description'] == nil then document.description.text = 'None' else document.description.text = packinfo['description'] end
end

function open_config()
    local id = pack_open[1]
    local path = "config:" .. id

    if (not document.configs.visible and id) and valid_configs(path) then
        document.configs:clear()
        document.configs.visible = true
        document.content_info.visible = false

        local configs = file.list("config:" .. id)

        for i, c in ipairs(configs) do
            local name = c:match("([^/]+)$")
            name = name:match("([^%.]+)")
            local config = load_config(c)

            if config then
                create_config(i, config, name, c)
            end
        end
        return
    end
    document.configs.visible = false
    document.content_info.visible = true
end

function refresh()
    local packs_available = pack.get_available()
    packs_installed = pack.get_installed()

    for _, k in ipairs(packs_available) do
        table.insert(packs_installed, k)
    end

    local contents = document.contents
    contents:clear()

    local packinfos = pack.get_info(packs_installed)
    for i, id in ipairs(packs_installed) do
        local packinfo = packinfos[id]
        packinfo.id = id
        packs_installed[i] = packinfo

        local callback = string.format('open_pack("%s")', id)
        place_pack(contents, packinfo, callback)
    end
end
