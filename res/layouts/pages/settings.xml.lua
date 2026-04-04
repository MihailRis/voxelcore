function on_open()
    document.langs_btn.text = string.format(
        "%s: %s", gui.str("Language", "settings"),
        gui.get_locales_info()[app.get_setting("ui.language")].name
    )
    sections = RadioGroup({
        audio=document.s_aud,
        display=document.s_dsp,
        graphics=document.s_gfx,
        controls=document.s_ctl,
        reset=document.s_rst
    }, function (page)
        document.menu.page = "settings_"..page
    end, "graphics")
end
