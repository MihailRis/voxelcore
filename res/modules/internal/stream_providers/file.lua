local io_stream = require "core:io_stream"

local lib = {
    read = file.__read_descriptor,
    write = file.__write_descriptor,
    flush = file.__flush_descriptor,
    is_alive = file.__has_descriptor,
    close = file.__close_descriptor
}

return function(path, mode)
    return io_stream.new(
        file.__open_descriptor(path, mode),
        mode:find('b') ~= nil,
        lib
    )
end