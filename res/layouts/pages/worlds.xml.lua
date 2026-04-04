function on_open()
    local worlds = world.get_list()
    for _, info in ipairs(worlds) do
        local major, minor = app.get_version()
        if info.version[1] > major or info.version[2] > minor then
            info.versionColor = "#A02010"
        else
            info.versionColor = "#808080"
        end
        info.versionString = string.format("%s.%s", unpack(info.version))
        document.worlds:add(gui.template("world", info))
    end
end
