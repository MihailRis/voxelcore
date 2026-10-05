local io_stream = require "core:io_stream"

local lib = {
    read = file.__read_descriptor,
    write = file.__write_descriptor,
    seek = file.__seek_descriptor,
    tell = file.__tell_descriptor,
    flush = file.__flush_descriptor,
    available = file.__available_descriptor,
    is_alive = file.__has_descriptor,
    close = file.__close_descriptor
}

local __open_descriptor = file.__open_descriptor
file.__open_descriptor = nil
file.__read_descriptor = nil
file.__write_descriptor = nil
file.__seek_descriptor = nil
file.__tell_descriptor = nil
file.__flush_descriptor = nil
file.__available_descriptor = nil
file.__has_descriptor = nil
file.__close_descriptor = nil

return function(path, mode)
    return io_stream.new(
        __open_descriptor(path, mode),
        mode:find('b') ~= nil,
        lib
    )
end
