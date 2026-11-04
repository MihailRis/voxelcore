local binary = require "internal/stream/binary"

local text = {}
text.__index = text

local CR = string.byte('\r')
local LF = string.byte('\n')

function text.new(stream)
    return setmetatable({stream = stream}, text)
end

function text:_read_byte()
    return self.stream:read(1)[1]
end

function text:read_line()
    local line = Bytearray()
    local got_any = false

    while true do
        local b = self:_read_byte()
        if b == nil then break end
        got_any = true

        if b == LF then
            break
        elseif b == CR then
            local nb = self:_read_byte()
            if nb == LF then break end
            line:append(CR)
            if nb == nil then break end
            line:append(nb)
        else
            line:append(b)
        end
    end

    if not got_any then return nil end
    return utf8.tostring(line)
end

local function trim_empty(lines)
    while #lines > 0 and lines[#lines] == "" do
        lines[#lines] = nil
    end
    while #lines > 0 and lines[1] == "" do
        table.remove(lines, 1)
    end
end

function text:read_lines(count, trim)
    if count < 0 then error("count of lines to read must be positive") end

    local lines = {}
    for i = 1, count do
        local line = self:read_line()
        if line == nil then break end
        lines[i] = line
    end

    if trim ~= false then trim_empty(lines) end
    return lines
end

function text:read(count, trim)
    if count == nil then
        return self:read_line()
    end
    return self:read_lines(count, trim)
end

function text:read_fully(as_lines)
    if as_lines then
        local lines = {}
        for line in self.read_line, self do
            lines[#lines + 1] = line
        end
        return lines
    end
    return utf8.tostring(binary.read_all(self.stream))
end

function text:write_line(str)
    self.stream:write(utf8.tobytes(str .. "\n"))
end

-- write(string) | write({ lines... })
function text:write(arg)
    local t = type(arg)
    if t == "string" then
        self:write_line(arg)
    elseif t == "table" then
        for i = 1, #arg do
            self:write_line(arg[i])
        end
    else
        error("unknown argument type: " .. t)
    end
end

return text
