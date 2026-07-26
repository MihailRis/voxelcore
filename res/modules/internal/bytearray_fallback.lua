-- Plain-Lua implementation of the Bytearray API from bytearray.lua.
-- Used when the FFI is unavailable (interpreter without LuaJIT) or
-- non-functional (no access to C symbols in the host binary).
-- Slower than the FFI version, but keeps scripts portable.

-- On Lua 5.1 the __len/__index metamethods for '#' and indexing only work
-- on userdata, which newproxy provides; newer interpreters dropped
-- newproxy but support the metamethods on plain tables.
local function make_object()
    if newproxy then
        local proxy = newproxy(true)
        return proxy, getmetatable(proxy)
    end
    local mt = {}
    return setmetatable({}, mt), mt
end

local function new_bytearray(init)
    local proxy, mt = make_object()
    local data = {}
    local size = 0

    local methods
    local function append(_, b)
        if type(b) == "number" then
            size = size + 1
            data[size] = b % 256
        else
            for i = 1, #b do
                size = size + 1
                data[size] = b[i] % 256
            end
        end
    end
    local function insert(_, index, b)
        if b == nil then
            b = index
            index = size + 1
        end
        if index <= 0 or index > size + 1 then
            return
        end
        local elems = type(b) == "number" and 1 or #b
        for i = size, index, -1 do
            data[i + elems] = data[i]
        end
        if type(b) == "number" then
            data[index] = b % 256
        else
            for i = 1, #b do
                data[index + i - 1] = b[i] % 256
            end
        end
        size = size + elems
    end
    local function remove(_, index, elems)
        if index <= 0 or index > size then
            return
        end
        elems = elems or 1
        if index + elems > size then
            elems = size - index + 1
        end
        for i = index, size - elems do
            data[i] = data[i + elems]
        end
        for i = size - elems + 1, size do
            data[i] = nil
        end
        size = size - elems
    end
    methods = {
        append = append,
        insert = insert,
        remove = remove,
        trim = function() end,
        clear = function() data = {}; size = 0 end,
        reserve = function() end,
        get_capacity = function() return size end,
        slice = function(_, offset, length)
            offset = offset or 1
            length = length or (size - offset + 1)
            local out = {}
            for i = 1, length do
                out[i] = data[offset + i - 1]
            end
            return new_bytearray(out)
        end,
        __is_bytearray = true,
        __data = function() return data, size end,
    }
    mt.__index = function(_, key)
        if type(key) == "string" then
            return methods[key]
        end
        if key <= 0 or key > size then
            return nil
        end
        return data[key]
    end
    mt.__newindex = function(_, key, value)
        if type(key) == "string" then
            return
        end
        if key == size + 1 then
            append(nil, value)
        elseif key >= 1 and key <= size then
            data[key] = value % 256
        end
    end
    mt.__len = function() return size end
    mt.__tostring = function()
        return string.format("Bytearray[%s]{...}", size)
    end
    local function iter()
        local i = 0
        return function()
            i = i + 1
            if i <= size then
                return i, data[i]
            end
        end
    end
    mt.__ipairs = iter
    mt.__pairs = iter

    if type(init) == "string" then
        for i = 1, #init do
            size = size + 1
            data[size] = init:byte(i)
        end
    elseif type(init) == "table" then
        for i = 1, #init do
            size = size + 1
            data[size] = init[i] % 256
        end
    elseif type(init) == "number" then
        for i = 1, init do
            data[i] = 0
        end
        size = init
    end
    return proxy
end

local FFIBytearray = setmetatable({}, {
    __call = function(_, n) return new_bytearray(n) end
})

local function as_string(bytes)
    if type(bytes) == "table" then
        local out = {}
        for i = 1, #bytes do
            out[i] = string.char(bytes[i] % 256)
        end
        return table.concat(out)
    end
    local data, size = bytes.__data()
    local out = {}
    for i = 1, size do
        out[i] = string.char(data[i])
    end
    return table.concat(out)
end

local function make_view(typesize, signed)
    return function(bytes)
        local view, mt = make_object()
        local function get_len()
            return math.floor(#bytes / typesize)
        end
        mt.__index = function(_, key)
            if key == "size" then
                return get_len()
            end
            if key <= 0 or key > get_len() then
                return nil
            end
            local base = (key - 1) * typesize
            local v = 0
            for i = typesize, 1, -1 do
                v = v * 256 + bytes[base + i]
            end
            if signed then
                local max = 2 ^ (typesize * 8 - 1)
                if v >= max then
                    v = v - max * 2
                end
            end
            return v
        end
        mt.__newindex = function(_, key, value)
            if key <= 0 or key > get_len() then
                return
            end
            local base = (key - 1) * typesize
            if value < 0 then
                value = value + 2 ^ (typesize * 8)
            end
            for i = 1, typesize do
                bytes[base + i] = value % 256
                value = math.floor(value / 256)
            end
        end
        mt.__len = get_len
        return view
    end
end

return {
    FFIBytearray = FFIBytearray,
    FFIBytearray_as_string = as_string,
    FFIBytearray_as_ptr = function() return "0" end,
    FFIU16view = make_view(2, false),
    FFII16view = make_view(2, true),
    FFIU32view = make_view(4, false),
    FFII32view = make_view(4, true),
}
