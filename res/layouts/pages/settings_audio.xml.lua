function create_setting(id, name, step, postfix)
    local info = core.get_setting_info(id)
    postfix = postfix or ""
    document.root:add(gui.template("track_setting", {
        id=id,
        name=gui.str(name, "settings"),
        value=core.get_setting(id),
        min=info.min,
        max=info.max,
        step=step,
        postfix=postfix
    }))
    update_setting(core.get_setting(id), id, name, postfix)
end

function update_setting(x, id, name, postfix)
    core.set_setting(id, x)
    -- updating label
    document[id..".L"].text = string.format(
        "%s: %s%s", 
        gui.str(name, "settings"), 
        core.str_setting(id), 
        postfix
    )
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
    create_setting("audio.volume-master", "Master Volume", 0.01)
    create_setting("audio.volume-regular", "Regular Sounds", 0.01)
    create_setting("audio.volume-ui", "UI Sounds", 0.01)
    create_setting("audio.volume-ambient", "Ambient", 0.01)
    create_setting("audio.volume-music", "Music", 0.01)
    document.root:add("<label context='settings'>@Microphone</label>")
    document.root:add("<select id='input_device_select' "..
        "onselect='function(opt) core.set_setting(\"audio.input-device\", opt) end'/>")
    document.root:add("<container id='input_volume_outer' color='#000000' size='4'>"
                        .."<container id='input_volume_inner' color='#00FF00FF' pos='1' size='2'/>"
                    .."</container>")
    local selectbox = document.input_device_select
    local devices = {
        {value="none", text=gui.str("None", "settings.microphone")},
    }
    local names = audio.__get_input_devices_names()
    for i, name in ipairs(names) do
        table.insert(devices, {value=name, text=name})
    end
    selectbox.options = devices
    selectbox.value = audio.__get_input_info().device_specifier
end
