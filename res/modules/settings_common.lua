-- Common helpers for settings pages (res/layouts/pages/settings_*.xml.lua).
--
-- Usage in a page script:
--
--   local settings = require "core:settings_common".new(document, {
--       app = app,
--       tostring_overrides = {["display.framerate"] = function(x) ... end}
--   })
--   -- templates call these functions by name, so they must be global
--   create_trackbar_setting = settings.create_trackbar_setting
--   update_trackbar_label = settings.update_trackbar_label
--   create_checkbox = settings.create_checkbox

local settings_common = {}

--- Create helpers bound to the page document.
--- @param document the page document
--- @param options table (optional) with fields:
---   app: the page's `app` library (not visible inside modules)
---   tostring_overrides: {[setting_id] = function(value) -> string}
function settings_common.new(document, options)
    options = options or {}
    local app = options.app
    local tostring_overrides = options.tostring_overrides or {}
    local this = {}

    --- Update trackbar label (called by the track_setting template)
    function this.update_trackbar_label(x, id, name, postfix)
        local str
        local func = tostring_overrides[id]
        if func then
            str = func(x)
        else
            str = app.str_setting(id)
        end
        document[id..".L"].text = string.format(
            "%s: %s%s",
            gui.str(name, "settings"),
            str,
            postfix
        )
    end

    --- Add a trackbar bound to the setting
    --- @param changeonrelease apply value only when mouse is released
    function this.create_trackbar_setting(
        id, name, step, postfix, tooltip, changeonrelease
    )
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
        this.update_trackbar_label(app.get_setting(id), id, name, postfix)
    end

    --- Add a checkbox bound to the setting
    --- @param consumer name of a global page function(id, value) called on
    --- change instead of just setting the value (optional)
    function this.create_checkbox(id, name, tooltip, consumer)
        tooltip = tooltip or ""
        local action
        if consumer then
            action = string.format("%s(\"%s\", x)", consumer, id)
        else
            action = string.format("app.set_setting(\"%s\", x)", id)
        end
        document.root:add(string.format(
            "<checkbox consumer='function(x) %s end' checked='%s' tooltip='%s'>%s</checkbox>",
            action, app.str_setting(id), gui.str(tooltip, "settings"), gui.str(name, "settings")
        ))
    end

    return this
end

return settings_common
