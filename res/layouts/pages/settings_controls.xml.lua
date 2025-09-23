local WARNING_COLORS = {
    {208, 104, 107, 255},
    {250, 75, 139, 255},
    {250, 151, 75, 255},
    {246, 233, 44, 255},
    {252, 200, 149, 255}
}

local GENERAL_WARNING_COLOR = {208, 138, 0, 255}

function refresh_search()
    local search_text = document.search_textbox.text
    local search_key = document.search_key_checkbox.checked

    local panel = document.bindings_panel
    local bindings = input.get_bindings()
    panel:clear()

    table.sort(bindings, function(a, b) return a > b end)
    if search_text ~= "" then
        for i,name in ipairs(bindings) do
            local _name = gui.str(name)
            if ((_name:lower():find(search_text:lower()) and not search_key) or
                (input.get_binding_text(name):lower():find(search_text:lower()) and search_key)) then
                panel:add(gui.template("binding", {
                    id=name, name=_name
                }))
            end
        end
    else
        for i,name in ipairs(bindings) do
            panel:add(gui.template("binding", {
                id=name, name=gui.str(name)
            }))
        end
    end
end

function refresh_sensitivity()
    document.sensitivity_label.text = string.format(
        "%s: %s", 
        gui.str("Mouse Sensitivity", "settings"),
        core.str_setting("camera.sensitivity")
    )
end

function change_sensitivity(val)
    core.set_setting("camera.sensitivity", val)
    refresh_sensitivity()
end

function refresh_binding_marks()
    local prev_bindings = {}
    local conflicts_colors = {}
    local available_colors = table.copy(WARNING_COLORS)

    local bindings = input.get_bindings()
    table.sort(bindings, function(a, b) return a > b end)

    for _, bind_name in ipairs(bindings) do
        local key = input.get_binding_text(bind_name)
        local prev = prev_bindings[key]
        if prev then
            local color = GENERAL_WARNING_COLOR
            local conflict_color = conflicts_colors[key]
            local available_colors_len = #available_colors
            if conflict_color then
                color = conflict_color
            elseif available_colors_len > 0 then
                color = available_colors[available_colors_len]
                conflicts_colors[key] = color
                table.remove(available_colors, available_colors_len)
            end

            local tooltip = gui.str("settings.Conflict", "settings")

            local prev_bindmark = "bindmark_" .. prev
            local cur_bindmark = "bindmark_" .. bind_name
            document[prev_bindmark].visible = true
            document[cur_bindmark].visible = true

            document[prev_bindmark].color = color
            document[cur_bindmark].color = color

            document["bind_" .. prev].tooltip = tooltip
            document["bind_" .. bind_name].tooltip = tooltip
        else
            document["bindmark_" .. bind_name].visible = false
            document["bind_" .. bind_name].tooltip = ''
            prev_bindings[key] = bind_name
        end
    end
end

function on_open()
    document.sensitivity_track.value = core.get_setting("camera.sensitivity")
    refresh_sensitivity()

    local panel = document.bindings_panel
    local bindings = input.get_bindings()
    table.sort(bindings, function(a, b) return a > b end)
    for i,name in ipairs(bindings) do
        panel:add(gui.template("binding", {
            id=name, name=gui.str(name)
        }))
    end

    document.bindings_panel:setInterval(100, function ()
        refresh_binding_marks()
    end)
end
