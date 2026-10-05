local io_stream = require "core:io_stream"

local lib = { }

local buffers = { }
local positions = { }

local next_descriptor = 0

local function open_descriptor(buffer)
    next_descriptor = next_descriptor + 1

    buffers[next_descriptor] = buffer
    positions[next_descriptor] = 1

    return next_descriptor
end

local function require_descriptor(descriptor)
    if not buffers[descriptor] then
        error("unknown descriptor")
    end
end

function lib.read(descriptor, length)
    require_descriptor(descriptor)

    local buf = buffers[descriptor]
    local buf_length = #buf
    local pos = positions[descriptor]

    local to_read = math.min(buf_length - pos + 1, length)

    if to_read <= 0 then
        return Bytearray()
    end

    local segment = buf:slice(pos, to_read)

    positions[descriptor] = pos + to_read

    return segment
end

function lib.write(descriptor, data)
    require_descriptor(descriptor)

    local buf = buffers[descriptor]
    local pos = positions[descriptor]

    local buf_length = #buf
    local data_length = #data

    local end_pos = pos + data_length - 1

    -- size ensuring
    if end_pos > buf_length then
        for i = buf_length + 1, end_pos do
            buf[i] = 0
        end
    end

    for i = 1, data_length do
        buf[i + pos - 1] = data[i]
    end

    positions[descriptor] = pos + data_length
end

function lib.seek(descriptor, mode, offset)
    require_descriptor(descriptor)

    local buf = buffers[descriptor]
    local buf_length = #buf

    local base

    if mode == 'b' then
        base = 1
    elseif mode == 'c' then
        base = positions[descriptor]
    elseif mode == 'e' then
        base = buf_length + 1
    else error('invalid seek mode') end

    local new_pos = base + offset

    if new_pos < 1 then
        error('failed to seek stream')
    end

    positions[descriptor] = new_pos
end

function lib.tell(descriptor)
    require_descriptor(descriptor)

    return positions[descriptor]
end

function lib.available(descriptor)
    require_descriptor(descriptor)

    local buf = buffers[descriptor]
    local pos = positions[descriptor]

    return math.max(#buf - pos + 1, 0)
end

function lib.is_alive(descriptor)
    return buffers[descriptor] ~= nil
end

function lib.close(descriptor)
    require_descriptor(descriptor)

    buffers[descriptor] = nil
    positions[descriptor] = nil
end

return function(buffer, binary_mode)
    if binary_mode == nil then
        binary_mode = true
    end

    return io_stream.new(
        open_descriptor(buffer),
        binary_mode,
        lib
    )
end