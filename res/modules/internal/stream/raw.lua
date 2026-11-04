local raw = {}
raw.__index = raw

function raw.new(descriptor, io_lib)
    return setmetatable({ descriptor = descriptor, io_lib = io_lib }, raw)
end

function raw:read(n)
    return self.io_lib.read(self.descriptor, n)
end

raw.read_some = raw.read

function raw:write(data)
    self.io_lib.write(self.descriptor, data)
end

function raw:flush()
    if self.io_lib.flush then
        self.io_lib.flush(self.descriptor)
    elseif not self:is_alive() then
        error("stream is closed")
    end
end

function raw:available()
    local f = self.io_lib.available
    return f and f(self.descriptor) or 0
end

function raw:seek(mode, offset)
    if not self.io_lib.seek then error("cannot seek this stream") end
    self.io_lib.seek(self.descriptor, mode, offset)
end

function raw:tell()
    if not self.io_lib.tell then error("cannot tell this stream") end
    return self.io_lib.tell(self.descriptor)
end

function raw:is_alive()
    return self.io_lib.is_alive(self.descriptor)
end

function raw:close()
    return self.io_lib.close(self.descriptor)
end

return raw
