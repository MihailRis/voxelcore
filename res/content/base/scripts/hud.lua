local base_util = require "util"

function on_hud_open()
    events.on("core:drop_outside_inventory", function(mode)
        local invid = hud.get_exchange_inventory()
        base_util.drop_from_slot(invid, 0, mode)
    end)
    input.add_callback("player.drop", function()
        if hud.is_paused() or hud.is_inventory_open() then
            return
        end
        local pid = hud.get_player()
        local invid, slot = player.get_inventory(pid)
        local mode = 1
        if input.is_pressed("key:left-ctrl") or input.is_pressed("key:right-ctrl") then
          mode = 0
        end
        base_util.drop_from_slot(invid, slot, mode)
    end)
    rules.create("do-loot-non-player", true)
end
