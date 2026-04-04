local updating_blocks = {}
local present_queues = {}
local REGISTER_BIT = 0x1
local UPDATING_BIT = 0x2
local PRESENT_BIT = 0x4
local REMOVED_BIT = 0x8

block.__perform_ticks = function(delta)
    for id, entry in pairs(updating_blocks) do
        entry.timer = entry.timer + delta
        local steps = math.floor(entry.timer / entry.delta * #entry / 3)
        steps = math.min(steps, #entry / 3)
        if steps == 0 then
            goto continue
        end
        entry.timer = 0.0
        entry.pointer = entry.pointer % #entry
        local event = entry.event
        local tps = entry.tps
        for i=1, steps do
            local x = entry[entry.pointer + 1]
            local y = entry[entry.pointer + 2]
            local z = entry[entry.pointer + 3]
            entry.pointer = (entry.pointer + 3) % #entry
            events.emit(event, x, y, z, tps)
        end
        ::continue::
    end
    for id, queue in pairs(present_queues) do
        queue.timer = queue.timer + delta
        local steps = math.floor(queue.timer / queue.delta * #queue / 3)
        steps = math.min(steps, #queue / 3)
        if steps == 0 then
            goto continue
        end
        queue.timer = 0.0
        local event = queue.event
        local update_list = updating_blocks[id]
        for i=1, steps do
            local index = #queue - 2
            if index <= 0 then
                break
            end
            local x = queue[index]
            local y = queue[index + 1]
            local z = queue[index + 2]

            for j=1,3 do
                table.remove(queue, index)
            end
            events.emit(event, x, y, z)

            if queue.updating then
                table.insert(update_list, x)
                table.insert(update_list, y)
                table.insert(update_list, z)
            end
        end
        ::continue::
    end
end

local block_pull_register_events = block.__pull_register_events
block.__pull_register_events = nil

block.__process_register_events = function()
    local register_events = block_pull_register_events()
    if not register_events then
        return
    end

    local emit_event = events.emit
    local removed_events = {}

    for i=1, #register_events, 4 do
        local header = register_events[i]
        local event_bits = bit.band(header, 0xFFFF)
        local id = bit.rshift(header, 16)
        local x = register_events[i + 1]
        local y = register_events[i + 2]
        local z = register_events[i + 3]

        local is_register = bit.band(event_bits, REGISTER_BIT) ~= 0
        local is_updating = bit.band(event_bits, UPDATING_BIT) ~= 0
        local is_present = bit.band(event_bits, PRESENT_BIT) ~= 0
        local is_removed = bit.band(event_bits, REMOVED_BIT) ~= 0
        local list = updating_blocks[id]

        if not is_register and is_removed then
            local rm_event = removed_events[id]
            if not rm_event then
                rm_event = block.name(id) .. ".blockremoved"
                removed_events[id] = rm_event
            end
            emit_event(rm_event, x, y, z)
        end

        if not list and is_register and is_updating then
            list = {}
            list.event = block.name(id) .. ".blocktick"
            list.tps = 20 / (block.properties[id]["tick-interval"] or 1)
            list.delta = 1.0 / list.tps
            list.timer = 0.0
            list.pointer = 0
            updating_blocks[id] = list
        end

        if is_register and is_present then
            local present_queue = present_queues[id]
            if not present_queue then
                present_queue = {}
                present_queue.event = block.name(id) .. ".blockpresent"
                present_queue.tps = 20 / (block.properties[id]["tick-interval"] or 1)
                present_queue.delta = 1.0 / present_queue.tps
                present_queue.timer = 0.0
                present_queue.pointer = 0
                present_queue.updating = is_updating
                present_queues[id] = present_queue
            end
            table.insert(present_queue, x)
            table.insert(present_queue, y)
            table.insert(present_queue, z)
            goto continue
        end
        if not is_updating then
            goto continue
        end
        if is_register then
            table.insert(list, x)
            table.insert(list, y)
            table.insert(list, z)
        else
            if not list then
                goto continue
            end
            for j=1, #list, 3 do
                if list[j] == x and list[j + 1] == y and list[j + 2] == z then
                    for k=1,3 do
                        table.remove(list, j)
                    end
                    j = j - 3
                end
            end
        end
        ::continue::
    end
end
