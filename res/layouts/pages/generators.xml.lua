settings = session.get('new_world')

function on_open()
    local names = generation.get_generators()
    local keys = {}
    for key in pairs(names) do
        table.insert(keys, key)
    end
    table.sort(keys)

    local panel = document.root
    for _, key in ipairs(keys) do
        local caption = names[key]
        panel:add(gui.template("generator", {
            callback=string.format("settings.generator=%q menu:back()", key),
            id=key,
            name=settings.generator_name(caption)
        }))
    end
    panel:add("<button onclick='menu:back()'>@Back</button>")
end
