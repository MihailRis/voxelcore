local menubg

function on_menu_clear()
    if menubg then
        menubg:destruct()
        menubg = nil
    end
end

local function setup_backround()
    local controller = {}
    function controller.resize_menu_bg()
        local w, h = unpack(gui.get_viewport())
        if menubg then
            menubg.region = {0, math.floor(h / 48), math.floor(w / 48), 0}
            menubg.pos = {0, 0}
        end
        return w, h
    end
    local bgid = random.uuid()
    gui.root.root:add(string.format(
        "<image id='%s' src='gui/menubg' size-func='DATA.resize_menu_bg' "..
        "z-index='-1' interactive='true'/>", bgid), controller)
    menubg = gui.root[bgid]
    controller.resize_menu_bg()
end

function on_menu_setup()
    setup_backround()
    menu.page = "main"
    menu.visible = true
end
