local wrapper = require "internal/stream/wrapper"

local buffered = wrapper.extend({})

buffered.DEFAULT_SIZE = 8192

function buffered.new(inner, size, flush_inner)
    return setmetatable({
        inner = inner,
        size = size or buffered.DEFAULT_SIZE,
        flush_inner = flush_inner ~= false,
        rbuf = Bytearray(),
        wbuf = Bytearray(),
    }, buffered)
end

function buffered:_take(n)
    local buf = self.rbuf
    if n >= #buf then
        self.rbuf = Bytearray()
        return buf
    end
    local part = buf:slice(1, n)
    buf:remove(1, n)
    return part
end

function buffered:flush_buffer()
    if #self.wbuf > 0 then
        local data = self.wbuf
        self.wbuf = Bytearray()
        self.inner:write(data)
    end
end

function buffered:read(n)
    local have = #self.rbuf
    if have < n then
        if have == 0 and n >= self.size then
            return self.inner:read(n)
        end
        self.rbuf:append(self.inner:read(self.size - have))
    end
    return self:_take(n)
end

buffered.read_some = buffered.read

function buffered:write(data)
    if #self.wbuf + #data > self.size then
        self:flush_buffer()
    end
    if #data >= self.size then
        self.inner:write(data)
    else
        self.wbuf:append(data)
    end
end

function buffered:flush()
    self:flush_buffer()
    if self.flush_inner then
        self.inner:flush()
    end
end

function buffered:available()
    return self.inner:available() + #self.rbuf
end

function buffered:seek(mode, offset)
    self:flush_buffer()
    if mode == "c" then
        offset = offset - #self.rbuf
    end
    self.rbuf = Bytearray()
    self.inner:seek(mode, offset)
end

function buffered:tell()
    return self.inner:tell() - #self.rbuf + #self.wbuf
end

function buffered:close()
    self:flush_buffer()
    self.rbuf = Bytearray()
    return self.inner:close()
end

return buffered
