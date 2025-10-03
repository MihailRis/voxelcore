local tostring_overrides = {}
tostring_overrides["display.framerate"] = function(x)
    if x == -1 then
        return gui.str("V-Sync")
    elseif x == 0 then
        return gui.str("Unlimited")
    else
        return tostring(x)
    end
end

function create_setting(id, name, step, postfix, tooltip, changeonrelease)
    local info = core.get_setting_info(id)
    postfix = postfix or ""
    tooltip = tooltip or ""
    changeonrelease = changeonrelease or ""
    document.root:add(gui.template("track_setting", {
        id=id,
        name=gui.str(name, "settings"),
        value=core.get_setting(id),
        min=info.min,
        max=info.max,
        step=step,
        postfix=postfix,
        tooltip=tooltip,
        changeonrelease=changeonrelease
    }))
    update_setting(core.get_setting(id), id, name, postfix)
end

function update_setting(x, id, name, postfix)
    local str
    local func = tostring_overrides[id]
    if func then
        str = func(x)
    else
        str = core.str_setting(id)
    end
    -- updating label
    document[id..".L"].text = string.format(
        "%s: %s%s", 
        gui.str(name, "settings"), 
        str, 
        postfix
    )
end

function create_checkbox(id, name, tooltip)
    tooltip = tooltip or ''
    document.root:add(string.format(
        "<checkbox consumer='function(x) core.set_setting(\"%s\", x) end' checked='%s' tooltip='%s'>%s</checkbox>", 
        id, core.str_setting(id), gui.str(tooltip, "settings"), gui.str(name, "settings")
    ))
end


function on_open()
    create_setting("camera.fov", "FOV", 1, "°")
    create_setting("display.framerate", "Framerate", 1, "", "", true)

    document.root:add(string.format(
        "<select context='settings' onselect='function(opt) core.set_setting(\"display.window-mode\", tonumber(opt)) end' selected='%s'>"..
            "<option value='0'>@Windowed</option>"..
            "<option value='1'>@Fullscreen</option>"..
            "<option value='2'>@Borderless</option>"..
        "</select>", core.get_setting("display.window-mode"))
    )

    create_checkbox("camera.shaking", "Camera Shaking")
    create_checkbox("camera.inertia", "Camera Inertia")
    create_checkbox("camera.fov-effects", "Camera FOV Effects")
    create_checkbox("display.limit-fps-iconified", "Limit Background FPS")
end
