local util = require "core:tests_util"

util.create_demo_world("core:default")

local pid = player.create("Xerxes")

local function selected_slot()
    local _, slot = player.get_inventory(pid)
    return slot
end

util.check_int(10, player.get_hotbar_size(pid), "default hotbar size")

player.set_selected_slot(pid, 12)
util.check_int(2, selected_slot(), "selected slot")

player.set_selected_slot(pid, 8)
player.set_hotbar_size(pid, 5)
util.check_int(5, player.get_hotbar_size(pid), "hotbar size")
util.check_int(4, selected_slot(), "clamped selected slot")

player.set_selected_slot(pid, 7)
util.check_int(2, selected_slot(), "selected slot")

player.set_hotbar_size(pid, 0)
util.check_int(1, player.get_hotbar_size(pid), "min hotbar size")
util.check_int(0, selected_slot(), "selected slot")
