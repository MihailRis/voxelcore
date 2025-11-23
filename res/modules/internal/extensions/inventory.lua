function inventory.get_uses(invid, slot)
    local uses = inventory.get_data(invid, slot, "uses")
    if uses == nil then
        return item.uses(inventory.get(invid, slot))
    end
    return uses
end

function inventory.use(invid, slot)
    local itemid, count = inventory.get(invid, slot)
    if itemid == nil then
        return
    end
    local item_uses = inventory.get_uses(invid, slot)
    if item_uses == nil then
        return
    end
    if item_uses == 1 then
        inventory.set(invid, slot, itemid, count - 1)
    elseif item_uses > 1 then
        inventory.set_data(invid, slot, "uses", item_uses - 1)
    end
end

function inventory.decrement(invid, slot, count)
    count = count or 1
    local itemid, itemcount = inventory.get(invid, slot)
    if itemcount <= count then
        inventory.set(invid, slot, 0)
    else
        inventory.set_count(invid, slot, itemcount - count)
    end
end

function inventory.get_caption(invid, slot)
    local item_id, count = inventory.get(invid, slot)
    local caption = inventory.get_data(invid, slot, "caption")
    if not caption then return item.caption(item_id) end

    return caption
end

function inventory.set_caption(invid, slot, caption)
    local itemid, itemcount = inventory.get(invid, slot)
    if itemid == 0 then
        return
    end
    if caption == nil or type(caption) ~= "string" then
        caption = ""
    end
    inventory.set_data(invid, slot, "caption", caption)
end

function inventory.get_description(invid, slot)
    local item_id, count = inventory.get(invid, slot)
    local description = inventory.get_data(invid, slot, "description")
    if not description then return item.description(item_id) end

    return description
end

function inventory.set_description(invid, slot, description)
    local itemid, itemcount = inventory.get(invid, slot)
    if itemid == 0 then
        return
    end
    if description == nil or type(description) ~= "string" then
        description = ""
    end
    inventory.set_data(invid, slot, "description", description)
end
