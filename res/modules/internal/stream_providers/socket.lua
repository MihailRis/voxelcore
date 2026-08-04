local io_stream = require "core:io_stream"

local lib = { }

local sockets = { }

local next_descriptor = 0

local function open_descriptor(socket)
    next_descriptor = next_descriptor + 1

    sockets[next_descriptor] = socket

    return next_descriptor
end

local function require_descriptor(descriptor)
    if not sockets[descriptor] then
        error("unknown descriptor")
    end
end

function lib.read(descriptor, length)
    require_descriptor(descriptor)

    return sockets[descriptor]:recv(length)
end

function lib.write(descriptor, data)
    require_descriptor(descriptor)

    sockets[descriptor]:send(data)
end

function lib.available(descriptor)
    require_descriptor(descriptor)

    return sockets[descriptor]:available()
end

function lib.is_alive(descriptor)
    local socket = sockets[descriptor]

    return socket ~= nil and socket:is_alive()
end

function lib.close(descriptor)
    require_descriptor(descriptor)

    sockets[descriptor]:close()
    sockets[descriptor] = nil
end

return function(socket, binary_mode)
    if binary_mode == nil then
        binary_mode = true
    end

    return io_stream.new(
        open_descriptor(socket),
        binary_mode,
        lib
    )
end