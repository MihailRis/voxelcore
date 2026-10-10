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

tostring_overrides["display.gui-scale"] = function(x)
    if x == 0 then
        -- the scale chosen automatically for the current window
        return string.format("%s (%s)", gui.str("Auto"), tostring(gui.get_scale()))
    end
    return tostring(x)
end

local settings = require "core:settings_common".new(document, {app=app, tostring_overrides=tostring_overrides})
-- templates call these functions by name, so they must be global
create_trackbar_setting = settings.create_trackbar_setting
update_trackbar_label = settings.update_trackbar_label
create_checkbox = settings.create_checkbox

function on_open()
    create_trackbar_setting("camera.fov", "FOV", 1, "°")
    create_trackbar_setting("display.framerate", "Framerate", 1, "", "", true)
    create_trackbar_setting("display.gui-scale", "GUI Scale", 0.5, "", "display.gui-scale.tooltip", true)

    document.root:add(string.format(
        "<select context='settings' onselect='function(opt) app.set_setting(\"display.window-mode\", tonumber(opt)) end' selected='%s'>"..
            "<option value='0'>@Windowed</option>"..
            "<option value='1'>@Fullscreen</option>"..
            "<option value='2'>@Borderless</option>"..
        "</select>", app.get_setting("display.window-mode"))
    )

    create_checkbox("camera.shaking", "Camera Shaking")
    create_checkbox("camera.inertia", "Camera Inertia")
    create_checkbox("camera.fov-effects", "Camera FOV Effects")
    create_checkbox("display.limit-fps-iconified", "Limit Background FPS")
    create_trackbar_setting("graphics.gamma", "Gamma", 0.05, "", "graphics.gamma.tooltip")
end
