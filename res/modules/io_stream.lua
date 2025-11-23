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

local function readFully(result, readFunc)
    local isTable = type(result) == "table"

    local buf

    repeat
        buf = readFunc(MAX_BUFFER_SIZE)

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
    write(descriptor: int, data: Bytearray)
    flush(descriptor: int)
    is_alive(descriptor: int) -> bool
    close(descriptor: int)
--]]

function io_stream.new(descriptor, binaryMode, ioLib, mode, flushMode)
    mode = mode or DEFAULT_MODE
    flushMode = flushMode or FLUSH_MODE_ALL

    local self = setmetatable({}, io_stream)

    self.descriptor = descriptor
    self.binaryMode = binaryMode
    self.maxBufferSize = MAX_BUFFER_SIZE
    self.ioLib = ioLib

    self:set_mode(mode)
    self:set_flush_mode(flushMode)

    return self
end

function io_stream:is_binary_mode()
    return self.binaryMode
end

function io_stream:set_binary_mode(binaryMode)
    self.binaryMode = binaryMode ~= nil
end

function io_stream:get_mode()
    return self.mode
end

function io_stream:set_mode(mode)
    if not table.has(ALL_MODES, mode) then
        error("invalid stream mode: "..mode)
    end

    if self.mode == BUFFERED_MODE then
        self.writeBuffer:clear()
        self.readBuffer:clear()
    end

    if mode == BUFFERED_MODE and not self.writeBuffer then
        self.writeBuffer = Bytearray()
        self.readBuffer = Bytearray()
    end

    self.mode = mode
end

function io_stream:get_flush_mode()
    return self.flushMode
end

function io_stream:set_flush_mode(flushMode)
    if not table.has(ALL_FLUSH_MODES, flushMode) then
        error("invalid flush mode: "..flushMode)
    end

    self.flushMode = flushMode
end

function io_stream:get_max_buffer_size()
    return self.maxBufferSize
end

function io_stream:set_max_buffer_size(maxBufferSize)
    self.maxBufferSize = maxBufferSize
end

function io_stream:available(length)
    if self.mode == BUFFERED_MODE then
        self:__update_read_buffer()

        if not length then
            return #self.readBuffer
        else
            return #self.readBuffer >= length
        end
    end
end

function io_stream:__update_read_buffer()
    local readed = Bytearray()

    readFully(readed, function(length) return self.ioLib.read(self.descriptor, length) end)

    self.readBuffer:append(readed)

    if #self.readBuffer > self.maxBufferSize then
        error "buffer overflow"
    end
end

function io_stream:__read(length)
    if self.mode == YIELD_MODE then
        local buffer = Bytearray()

        while #buffer < length do
            buffer:append(self.ioLib.read(self.descriptor, length - #buffer))

            if #buffer < length then coroutine.yield() end
        end

        return buffer
    elseif self.mode == BUFFERED_MODE then
        self:__update_read_buffer()

        if #self.readBuffer < length then
            error "buffer underflow"
        end

        local copy

        if #self.readBuffer == length then
            copy = Bytearray()
            
            copy:append(self.readBuffer)

            self.readBuffer:clear()
        else
            copy = Bytearray()

            for i = 1, length do
                copy[i] = self.readBuffer[i]
            end

            self.readBuffer:remove(1, length)
        end

        return copy
    elseif self.mode == DEFAULT_MODE then
        return self.ioLib.read(self.descriptor, length)
    end
end

function io_stream:__write(data)
    if self.mode == BUFFERED_MODE then
        self.writeBuffer:append(data)

        if #self.writeBuffer > self.maxBufferSize then
            error "buffer overflow"
        end
    elseif self.mode == DEFAULT_MODE or self.mode == YIELD_MODE then
        return self.ioLib.write(self.descriptor, data)
    end
end

function io_stream:read_fully(useTable)
    if self.binaryMode then
        local result = useTable and Bytearray() or { }

        readFully(result, function() return self:__read(self.maxBufferSize) end)
    else
        if useTable then
            local lines = { }

            local line

            repeat
                line = self:read_line()

                lines[#lines + 1] = line
            until not line

            return lines
        else
            local result = Bytearray()

            readFully(result, function() return self:__read(self.maxBufferSize) end)

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
    self:__write(utf8.tobytes(str .. LF))
end

function io_stream:read(arg, useTable)
    local argType = type(arg)

    if self.binaryMode then
        local byteArr

        if argType == "number" then
            -- using 'arg' as length

            byteArr = self:__read(arg)

            if useTable == true then
                local t = { }

                for i = 1, #byteArr do
                    t[i] = byteArr[i]
                end

                return t
            else
                return byteArr
            end
        elseif argType == "string" then
            return byteutil.unpack(
                arg,
                self:__read(byteutil.get_size(arg))
            )
        elseif argType == nil then
            error(
                "in binary mode the first argument must be a string data format"..
                " for the library \"byteutil\" or the number of bytes to read"
            )
        else
            error("unknown argument type: "..argType)
        end
    else
        if not arg then
            return self:read_line()
        else
            local linesCount = arg
            local trimLastEmptyLines = useTable or true

            if linesCount < 0 then error "count of lines to read must be positive" end

            local result = { }

            for i = 1, linesCount do
                result[i] = self:read_line()
            end

            if trimLastEmptyLines then
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
    local argType = type(arg)

    if self.binaryMode then
        local byteArr

        if argType ~= "string" then
            -- using arg as bytes table/bytearray

            if argType == "table" then
                byteArr = Bytearray(arg)
            else
                byteArr = arg
            end
        else
            byteArr = byteutil.pack(arg, ...)
        end

        self:__write(byteArr)
    else
        if argType == "string" then
            self:write_line(arg)
        elseif argType == "table" then
            for i = 1, #arg do
                self:write_line(arg[i])
            end
        else error("unknown argument type: "..argType) end
    end
end

function io_stream:is_alive()
    return self.ioLib.is_alive(self.descriptor)
end

function io_stream:is_closed()
    return not self:is_alive()
end

function io_stream:close()
    if self.mode == BUFFERED_MODE then
        self.readBuffer:clear()
        self.writeBuffer:clear()
    end

    return self.ioLib.close(self.descriptor)
end

function io_stream:flush()
    if self.mode == BUFFERED_MODE and #self.writeBuffer > 0 then
        self.ioLib.write(self.descriptor, self.writeBuffer)
        self.writeBuffer:clear()
    end

    if self.flushMode ~= FLUSH_MODE_ONLY_BUFFER then self.ioLib.flush(self.descriptor) end
end

return io_stream