local settings = require "core:settings_common".new(document, {app=app})
-- templates call these functions by name, so they must be global
create_trackbar_setting = settings.create_trackbar_setting
update_trackbar_label = settings.update_trackbar_label
create_checkbox = settings.create_checkbox

function on_open()
    create_trackbar_setting("chunks.load-distance", "Load Distance", 1)
    create_trackbar_setting("chunks.load-speed", "Load Speed", 1)
    create_trackbar_setting("graphics.fog-curve", "Fog Curve", 0.1)

    create_checkbox("graphics.enable-fog", "Fog", "graphics.enable-fog.tooltip")
    create_checkbox("graphics.backlight", "Backlight", "graphics.backlight.tooltip")
    create_checkbox("graphics.soft-lighting", "Soft lighting", "graphics.soft-lighting.tooltip")
    create_checkbox("graphics.dense-render", "Dense blocks render", "graphics.dense-render.tooltip")
    create_checkbox("graphics.advanced-render", "Advanced render", "graphics.advanced-render.tooltip")
    create_trackbar_setting("graphics.ssao", "SSAO", 1, "", "graphics.ssao.tooltip")
    create_trackbar_setting("graphics.shadows-quality", "Shadows quality", 1)
    create_trackbar_setting("graphics.clouds-quality", "Clouds quality", 1)
end
