function on_block_break_by(x, y, z, pid)
    block.set(x, y, z, 0, 0)
    if not player.is_infinite_items(pid) then
        inventory.use(player.get_inventory(pid))
    end
end
