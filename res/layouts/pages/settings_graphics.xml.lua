function create_trackbar_setting(id, name, step, postfix, tooltip, changeonrelease)
    local info = app.get_setting_info(id)
    postfix = postfix or ""
    tooltip = tooltip or ""
    changeonrelease = changeonrelease or ""
    document.root:add(gui.template("track_setting", {
        id=id,
        name=gui.str(name, "settings"),
        value=app.get_setting(id),
        min=info.min,
        max=info.max,
        step=step,
        postfix=postfix,
        tooltip=tooltip,
        changeonrelease=changeonrelease
    }))
    update_trackbar_label(app.get_setting(id), id, name, postfix)
end

function update_trackbar_label(x, id, name, postfix)
    -- updating label
    document[id..".L"].text = string.format(
        "%s: %s%s",
        gui.str(name, "settings"),
        app.str_setting(id),
        postfix
    )
end

function create_checkbox(id, name, tooltip)
    tooltip = tooltip or ''
    document.root:add(string.format(
        "<checkbox consumer='function(x) app.set_setting(\"%s\", x) end' checked='%s' tooltip='%s'>%s</checkbox>",
        id, app.str_setting(id), gui.str(tooltip, "settings"), gui.str(name, "settings")
    ))
end

function on_open()
    create_trackbar_setting("chunks.load-distance", "Load Distance", 1)
    create_trackbar_setting("chunks.load-speed", "Load Speed", 1)
    create_trackbar_setting("graphics.fog-curve", "Fog Curve", 0.1)

    create_checkbox("graphics.backlight", "Backlight", "graphics.backlight.tooltip")
    create_checkbox("graphics.soft-lighting", "Soft lighting", "graphics.soft-lighting.tooltip")
    create_checkbox("graphics.dense-render", "Dense blocks render", "graphics.dense-render.tooltip")
    create_checkbox("graphics.advanced-render", "Advanced render", "graphics.advanced-render.tooltip")
    create_trackbar_setting("graphics.ssao", "SSAO", 1, "", "graphics.ssao.tooltip")
    create_trackbar_setting("graphics.shadows-quality", "Shadows quality", 1)
    create_trackbar_setting("graphics.clouds-quality", "Clouds quality", 1)
end
