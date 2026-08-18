local M = {}

function M.share_default_func(invid, slotid)
    local blockinv = hud.get_block_inventory()
    local playerinv = player.get_inventory(hud.get_player())
    if blockinv ~= 0 then
        if invid == blockinv then
            inventory.move(blockinv, slotid, playerinv)
        else
            inventory.move(invid, slotid, blockinv)
        end
    elseif rules.get("allow-content-access") then
        inventory.set(invid, slotid, 0, 0)
    elseif slotid < 10 then
        inventory.move_range(invid, slotid, invid, 10)
    else
        inventory.move_range(invid, slotid, invid, 0, 9)
    end
end

return M
