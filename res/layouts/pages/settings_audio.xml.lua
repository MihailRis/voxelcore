local settings = require "core:settings_common".new(document, {})
-- templates call these functions by name, so they must be global
create_trackbar_setting = settings.create_trackbar_setting
update_trackbar_label = settings.update_trackbar_label

function update_checkbox_setting(id, value)
    app.set_setting(id, value)

    local recording_enabled = app.get_setting("audio.recording-enabled")
    document.input_device_select.enabled = recording_enabled
    document.input_volume_inner.enabled = recording_enabled
    document.input_volume_outer.enabled = recording_enabled

    local selectbox = document.input_device_select
    local info = audio.input.get_input_info()
    if info then
        selectbox.value = info.device_specifier
    else
        selectbox.value = app.get_setting("audio.input-device")
    end
end

local function create_audio_checkbox(id, name, tooltip)
    settings.create_checkbox(id, name, tooltip, "update_checkbox_setting")
    update_checkbox_setting(id, app.get_setting(id))
end

local initialized = false

function on_open()
    if not initialized then
        initialized = true
        local token = core.get_core_token()
        document.root:add("<container id='tm' />")
        local prev_amplitude = 0.0
        document.tm:setInterval(16, function()
            audio.input.fetch(token)
            local amplitude = audio.input.get_max_amplitude()
            if amplitude > 0.0 then
                amplitude = math.sqrt(amplitude)
            end
            amplitude = math.max(amplitude, prev_amplitude - time.delta())
            document.input_volume_inner.size = {
                prev_amplitude *
                document.input_volume_outer.size[1],
                document.input_volume_outer.size[2]
            }
            prev_amplitude = amplitude
        end)
    end
    create_trackbar_setting("audio.volume-master", "Master Volume", 0.01)
    create_trackbar_setting("audio.volume-regular", "Regular Sounds", 0.01)
    create_trackbar_setting("audio.volume-ui", "UI Sounds", 0.01)
    create_trackbar_setting("audio.volume-ambient", "Ambient", 0.01)
    create_trackbar_setting("audio.volume-music", "Music", 0.01)
    create_trackbar_setting("audio.volume-contrast", "Volume Contrast", 0.01, "", "audio.volume-contrast.tooltip")

    document.root:add("<label context='settings'>@Microphone</label>")
    document.root:add("<select id='input_device_select' "..
        "onselect='function(opt) app.set_setting(\"audio.input-device\", opt) end'/>")
    document.root:add("<container id='input_volume_outer' color='#000000' size='4'>"
                        .."<container id='input_volume_inner' color='#00FF00FF' pos='1' size='2'/>"
                    .."</container>")
    local selectbox = document.input_device_select
    local devices = {}
    local names = audio.__get_input_devices_names()
    for i, name in ipairs(names) do
        table.insert(devices, {value=name, text=name})
    end
    selectbox.options = devices

    local info = audio.input.get_input_info()
    if info then
        selectbox.value = info.device_specifier
    else
        selectbox.value = app.get_setting("audio.input-device")
    end

    create_audio_checkbox("audio.recording-enabled", "Microphone access", "audio.recording-enabled.tooltip")
    create_audio_checkbox("audio.acoustic-effects", "Acoustic effects", "audio.acoustic-effects.tooltip")
end
