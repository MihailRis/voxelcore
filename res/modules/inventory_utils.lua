local M = {}

function M.share_default_func(invid, slotid)
    local blockinv = hud.get_block_inventory()
    local pid = hud.get_player()
    local playerinv = player.get_inventory(pid)
    local hotbar_size = player.get_hotbar_size(pid)
    if blockinv ~= 0 then
        if invid == blockinv then
            inventory.move(blockinv, slotid, playerinv)
        else
            inventory.move(invid, slotid, blockinv)
        end
    elseif rules.get("allow-content-access") then
        inventory.set(invid, slotid, 0, 0)
    elseif slotid < hotbar_size then
        inventory.move_range(invid, slotid, invid, hotbar_size)
    else
        inventory.move_range(invid, slotid, invid, 0, hotbar_size - 1)
    end
end

return M
