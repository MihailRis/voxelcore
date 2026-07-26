local util = {}

function util.drop(ppos, itemid, count, data, pickup_delay)
    if itemid == 0 or not itemid then
        return nil
    end
    return entities.spawn("base:drop", ppos, {base__drop={
        id=itemid,
        count=count,
        data=data,
        pickup_delay=pickup_delay
    }})
end

local function calc_loot(loot_table)
    local results = {}
    for _, loot in ipairs(loot_table) do
        local chance = loot.chance or 1
        local count = loot.count or 1
        
        local roll = math.random()
        
        if roll < chance then
            if loot.min and loot.max then
                count = math.random(loot.min, loot.max)
            end
            if count ~= 0 then
                table.insert(results, {item=item.index(loot.item), count=count})
            end
        end
    end
    return results
end

function util.block_loot(blockid)
    local lootscheme = block.properties[blockid]["base:loot"]
    if lootscheme then
        return calc_loot(lootscheme)
    end
    return {{item=block.get_picking_item(blockid), count=1}}
end

return util
