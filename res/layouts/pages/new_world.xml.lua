settings = session.get_entry('new_world')

function world_name_validator(name)
    return name:match("^[%w-\\.\\ ]+$") ~= nil and not world.exists(name)
end

function save_state()
    settings.name = document.name_box.text
    settings.seed = document.seed_box.text
end

function settings.generator_name(id)
    return gui.str(id, "world.generators"):gsub("^%l", string.upper)
end

function create_world()
    if not document.name_box.valid then
        return
    end
    local name = document.name_box.text
    local seed = document.seed_box.text
    local generator = settings.generator
    session.reset_entry('new_world')
    core.new_world(name, seed, generator)
end

function world_name_placeholder()
    local name = "New World"
    local number = 0
    while world.exists(name) do
        number = number + 1
        name = "New World " .. tostring(number)
    end
    return name
end

function on_open()
    document.content_btn.text = string.format(
        "%s [%s]", gui.str("Content", "menu"), #pack.get_installed()
    )

    if settings.generator == nil or generation.get_generators()[settings.generator] == nil then
        settings.generator = generation.get_default_generator()
    end
    document.generator_btn.text = string.format(
        "%s: %s",
        gui.str("World generator", "world"),
        settings.generator_name(generation.get_generators()[settings.generator])
    )
    document.name_box.text = settings.name or ''
    document.name_box.placeholder = world_name_placeholder()
    document.seed_box.text = settings.seed or ''
    document.seed_box.placeholder = tostring(math.random()):sub(3)
end
