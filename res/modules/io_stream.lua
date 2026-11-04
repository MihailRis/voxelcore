local raw      = require "internal/stream/raw"
local buffered = require "internal/stream/buffered"
local blocking = require "internal/stream/blocking"
local binary   = require "internal/stream/binary"
local text     = require "internal/stream/text"

local io_stream = {}
io_stream.__index = io_stream

local MODES = { default = true, buffered = true, yield = true }
local FLUSH_MODES = { all = true, buffer = true }

function io_stream.new(descriptor, binary_mode, io_lib, mode, flush_mode)
    local self = setmetatable({
        descriptor = descriptor,
        io_lib = io_lib,
        binary_mode = binary_mode and true or false,
        max_buffer_size = buffered.DEFAULT_SIZE,
    }, io_stream)

    self:set_flush_mode(flush_mode or "all")
    self:set_mode(mode or "default")
    return self
end

function io_stream:_rebuild()
    local old = self.stream
    if old and old.flush_buffer and old:is_alive() then
        old:flush_buffer()
    end

    local s = raw.new(self.descriptor, self.io_lib)
    if self.mode == "buffered" then
        s = buffered.new(s, self.max_buffer_size, self.flush_mode == "all")
    elseif self.mode == "yield" then
        s = blocking.new(s)
    end

    self.stream = s
    self.text = text.new(s)
    self.binary = binary.new(s)
end

function io_stream:_format()
    return self.binary_mode and self.binary or self.text
end

function io_stream:is_binary_mode() return self.binary_mode end
function io_stream:set_binary_mode(v) self.binary_mode = v and true or false end

function io_stream:get_mode() return self.mode end

function io_stream:set_mode(mode)
    if not MODES[mode] then error("invalid stream mode: " .. tostring(mode)) end
    self.mode = mode
    self:_rebuild()
end

function io_stream:get_flush_mode() return self.flush_mode end

function io_stream:set_flush_mode(flush_mode)
    if not FLUSH_MODES[flush_mode] then
        error("invalid flush mode: " .. tostring(flush_mode))
    end
    self.flush_mode = flush_mode
    if self.mode == "buffered" then
        self.stream.flush_inner = flush_mode == "all"
    end
end

function io_stream:get_max_buffer_size() return self.max_buffer_size end

function io_stream:set_max_buffer_size(size)
    self.max_buffer_size = size
    if self.mode == "buffered" then self:_rebuild() end
end

function io_stream:read(arg, use_table) return self:_format():read(arg, use_table) end

function io_stream:write(arg, ...) return self:_format():write(arg, ...) end

function io_stream:read_fully(use_table) return self:_format():read_fully(use_table) end

function io_stream:read_line() return self.text:read_line() end

function io_stream:write_line(str) return self.text:write_line(str) end

function io_stream:available(length)
    local n = self.stream:available()
    if length then return n >= length end
    return n
end

function io_stream:seek(mode, offset) return self.stream:seek(mode, offset) end

function io_stream:tell() return self.stream:tell() end

function io_stream:is_alive() return self.stream:is_alive() end

function io_stream:is_closed() return not self.stream:is_alive() end

function io_stream:close() return self.stream:close() end

function io_stream:flush()
    if self.mode ~= "buffered" and self.flush_mode == "buffer" then return end
    self.stream:flush()
end
return io_stream
