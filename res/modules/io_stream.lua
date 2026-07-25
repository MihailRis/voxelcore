local io_stream = { }

io_stream.__index = io_stream

local MAX_BUFFER_SIZE = 8192

local DEFAULT_MODE = "default"
local BUFFERED_MODE = "buffered"
local YIELD_MODE = "yield"

local ALL_MODES = {
    DEFAULT_MODE,
    BUFFERED_MODE,
    YIELD_MODE
}

local FLUSH_MODE_ALL = "all"
local FLUSH_MODE_ONLY_BUFFER = "buffer"

local ALL_FLUSH_MODES = {
    FLUSH_MODE_ALL,
    FLUSH_MODE_ONLY_BUFFER
}

local CR = string.byte('\r')
local LF = string.byte('\n')

local function read_fully(result, read_func)
    local isTable = type(result) == "table"

    local buf

    repeat
        buf = read_func(MAX_BUFFER_SIZE)

        if isTable then
            for i = 1, #buf do
                result[#result + 1] = buf[i]
            end
        else result:append(buf) end
    until #buf == 0
end

--[[
descriptor - descriptor of stream for provided I/O library
binaryMode - if enabled, most methods will expect bytes instead of strings
ioLib - I/O library. Should include the following functions:
    read(descriptor: int, length: int) -> Bytearray
        May return bytearray with a smaller size if bytes have not arrived yet or have run out
        May throw error if descriptor isn't readable
    write(descriptor: int, data: Bytearray)
        May throw error if descriptor isn't writeable
    [optional] seek(descriptor: int, mode: string, offset: int)
        Mode may be 'b' (relative begin), 'c' (relative current), 'e' (relative end + 1)
    [optional] tell(descriptor: int) -> int
    [optional] flush(descriptor: int)
    [optional] available(descriptor: int) -> int
    is_alive(descriptor: int) -> bool
    close(descriptor: int)
--]]

function io_stream.new(descriptor, binary_mode, io_lib, mode, flush_mode)
    mode = mode or DEFAULT_MODE
    flush_mode = flush_mode or FLUSH_MODE_ALL

    local self = setmetatable({}, io_stream)

    self.descriptor = descriptor
    self.binary_mode = binary_mode
    self.max_buffer_size = MAX_BUFFER_SIZE
    self.io_lib = io_lib

    self:set_mode(mode)
    self:set_flush_mode(flush_mode)

    return self
end

function io_stream:is_binary_mode()
    return self.binary_mode
end

function io_stream:set_binary_mode(binary_mode)
    self.binary_mode = binary_mode ~= nil
end

function io_stream:get_mode()
    return self.mode
end

function io_stream:set_mode(mode)
    if not table.has(ALL_MODES, mode) then
        error("invalid stream mode: "..mode)
    end

    if self.write_buffer then
        self.write_buffer:clear()
        self.write_buffer = nil
    end

    if self.read_buffer then
        self.read_buffer:clear()
        self.read_buffer = nil
    end

    if mode == BUFFERED_MODE then
        self.write_buffer = Bytearray(self.max_buffer_size)
        self.read_buffer = Bytearray(self.max_buffer_size)
    end

    self.mode = mode
end

function io_stream:get_flush_mode()
    return self.flush_mode
end

function io_stream:set_flush_mode(flush_mode)
    if not table.has(ALL_FLUSH_MODES, flush_mode) then
        error("invalid flush mode: " .. flush_mode)
    end

    self.flush_mode = flush_mode
end

function io_stream:get_max_buffer_size()
    return self.max_buffer_size
end

function io_stream:set_max_buffer_size(max_buffer_size)
    self.max_buffer_size = max_buffer_size

    self.write_buffer = Bytearray(self.max_buffer_size)
    self.read_buffer = Bytearray(self.max_buffer_size)
end

function io_stream:available(length)
    local available = self.io_lib.available and self.io_lib.available(self.descriptor) or 0

    if self.mode == BUFFERED_MODE then
        available = available + #self.read_buffer
    end

    if not length then
        return available
    else
        return available >= length
    end
end

function io_stream:__read(length, from_read_fully)
    if self.mode == YIELD_MODE then
        if from_read_fully then
            return self.io_lib.read(self.descriptor, length)
        end

        local buffer = Bytearray()

        while #buffer < length do
            buffer:append(self.io_lib.read(self.descriptor, length - #buffer))

            if #buffer < length then coroutine.yield() end
        end

        return buffer
    elseif self.mode == BUFFERED_MODE then
        local buf_len = #self.read_buffer

        if buf_len < length then
            self.read_buffer:append(
                self.io_lib.read(self.descriptor, self.max_buffer_size - buf_len)
            )
        end

        buf_len = #self.read_buffer

        length = math.min(buf_len, length)

        local copy = self.read_buffer:slice(1, length)

        if buf_len == length then
            self.read_buffer:clear()
        else
            self.read_buffer:remove(1, length)
        end

        return copy
    elseif self.mode == DEFAULT_MODE then
        return self.io_lib.read(self.descriptor, length)
    end
end

function io_stream:__write(data)
    if self.mode == BUFFERED_MODE then
        local data_length = #data

        if #self.write_buffer + data_length > self.max_buffer_size then
            self:flush()
        end

        if data_length > self.max_buffer_size then
            local to_write = math.floor(data_length / self.max_buffer_size) * self.max_buffer_size
            local to_save = data_length - to_write

            self.io_lib.write(self.descriptor, data:slice(1, to_write))

            self:flush()

            self.write_buffer = data:slice(to_write + 1, to_save)
        else self.write_buffer:append(data) end
    elseif self.mode == DEFAULT_MODE or self.mode == YIELD_MODE then
        return self.io_lib.write(self.descriptor, data)
    end
end

function io_stream:read_fully(use_table)
    if self.binary_mode then
        local result = use_table and { } or Bytearray()

        local avail = self:available()

        if avail == 0 then
            avail = self.max_buffer_size
        end

        read_fully(result, function() return self:__read(avail, true) end)

        return result
    else
        if use_table then
            local lines = { }

            local line

            repeat
                line = self:read_line()

                lines[#lines + 1] = line
            until not line

            return lines
        else
            local result = Bytearray()

            read_fully(result, function() return self:__read(self.max_buffer_size) end)

            return utf8.tostring(result)
        end
    end
end

function io_stream:read_line()
    local result = Bytearray()

    local first = true

    while true do
        local char = self:__read(1)

        if #char == 0 then
            if first then return else break end
        end

        char = char[1]

        if char == LF then break
        elseif char == CR then
            char = self:__read(1)

            if char[1] == LF then break
            else
                result:append(CR)
                result:append(char[1])
            end
        else result:append(char) end

        first = false
    end

    return utf8.tostring(result)
end

function io_stream:write_line(str)
    self:__write(utf8.tobytes(str .. "\n"))
end

function io_stream:read(arg, use_table)
    local arg_type = type(arg)

    if self.binary_mode then
        local byte_arr

        if arg_type == "number" then
            -- using 'arg' as length

            byte_arr = self:__read(arg)

            if use_table == true then
                local t = { }

                for i = 1, #byte_arr do
                    t[i] = byte_arr[i]
                end

                return t
            else
                return byte_arr
            end
        elseif arg_type == "string" then
            return byteutil.unpack(
                arg,
                self:__read(byteutil.get_size(arg))
            )
        elseif arg_type == "nil" then
            error(
                "in binary mode the first argument must be a string data format"..
                " for the library \"byteutil\" or the number of bytes to read"
            )
        else
            error("unknown argument type: "..arg_type)
        end
    else
        if not arg then
            return self:read_line()
        else
            local lines_count = arg
            local trim_last_empty_lines = use_table

            if use_table == nil then
                trim_last_empty_lines = true
            end

            if lines_count < 0 then error "count of lines to read must be positive" end

            local result = { }

            for i = 1, lines_count do
                result[i] = self:read_line()
            end

            if trim_last_empty_lines then
                local i = #result

                while i >= 0 do
                    local length = utf8.length(result[i])

                    if length > 0 then break
                    else result[i] = nil end

                    i = i - 1
                end

                local i = 1

                while #result > 0 do
                    local length = utf8.length(result[i])

                    if length > 0 then break
                    else table.remove(result, i) end
                end
            end

            return result
        end
    end
end

function io_stream:write(arg, ...)
    local arg_type = type(arg)

    if self.binary_mode then
        local byte_arr

        if arg_type ~= "string" then
            -- using arg as bytes table/bytearray

            if arg_type == "table" then
                byte_arr = Bytearray(arg)
            else
                byte_arr = arg
            end
        else
            byte_arr = byteutil.pack(arg, ...)
        end

        self:__write(byte_arr)
    else
        if arg_type == "string" then
            self:write_line(arg)
        elseif arg_type == "table" then
            for i = 1, #arg do
                self:write_line(arg[i])
            end
        else error("unknown argument type: "..arg_type) end
    end
end

function io_stream:seek(mode, offset)
    if not self.io_lib.seek then
        error("cannot seek this stream")
    end

    self.io_lib.seek(self.descriptor, mode, offset)
end

function io_stream:tell()
    if not self.io_lib.tell then
        error("cannot tell this stream")
    end

    return self.io_lib.tell(self.descriptor)
end

function io_stream:is_alive()
    return self.io_lib.is_alive(self.descriptor)
end

function io_stream:is_closed()
    return not self:is_alive()
end

function io_stream:close()
    if self.mode == BUFFERED_MODE then
        self.read_buffer:clear()
        self.write_buffer:clear()
    end

    return self.io_lib.close(self.descriptor)
end

function io_stream:flush()
    if self.mode == BUFFERED_MODE and #self.write_buffer > 0 then
        self.io_lib.write(self.descriptor, self.write_buffer)
        self.write_buffer:clear()
    end

    if self.flush_mode ~= FLUSH_MODE_ONLY_BUFFER then
        if self.io_lib.flush then
            self.io_lib.flush(self.descriptor)
        elseif self:is_closed() then error("stream is closed") end
    end
end

return io_stream