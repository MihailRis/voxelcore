local binary = {}
binary.__index = binary

local DEFAULT_CHUNK = 8192

function binary.new(stream)
    return setmetatable({ stream = stream }, binary)
end

local function to_table(bytes)
    local t = {}
    for i = 1, #bytes do
        t[i] = bytes[i]
    end
    return t
end

function binary.read_all(stream)
    local result = Bytearray()
    local avail = stream:available()
    local chunk = avail > 0 and avail or DEFAULT_CHUNK

    while true do
        local part = stream:read_some(chunk)
        if #part == 0 then break end
        result:append(part)
    end

    return result
end

function binary:read(arg, as_table)
    local t = type(arg)

    if t == "number" then
        local bytes = self.stream:read(arg)
        return as_table and to_table(bytes) or bytes
    elseif t == "string" then
        local size = byteutil.get_size(arg)
        local bytes = self.stream:read(size)
        if #bytes < size then
            error(string.format(
                "unexpected end of stream: format %q needs %d bytes, got %d",
                arg, size, #bytes), 2)
        end
        return byteutil.unpack(arg, bytes)
    elseif t == "nil" then
        error("in binary mode the first argument must be a byteutil format string"
            .. " or the number of bytes to read")
    else
        error("unknown argument type: " .. t)
    end
end

function binary:write(arg, ...)
    local t = type(arg)

    if t == "string" then
        self.stream:write(byteutil.pack(arg, ...))
    elseif t == "table" then
        self.stream:write(Bytearray(arg))
    else
        self.stream:write(arg)
    end
end

function binary:read_fully(as_table)
    local bytes = binary.read_all(self.stream)
    return as_table and to_table(bytes) or bytes
end

return binary
