local MIN_CAPACITY = 8
local _type = type
local FFI = ffi

FFI.cdef[[
    void* malloc(size_t);
    void free(void*);
    typedef struct {
        unsigned char* bytes;
        int size;
        int capacity;
    } bytearray_t;
]]

local malloc = FFI.C.malloc
local free = FFI.C.free

local function grow_buffer(self, elems)
    local new_capacity = math.ceil(self.capacity / 0.75 + elems)
    local prev = self.bytes
    self.bytes = malloc(new_capacity)
    FFI.copy(self.bytes, prev, self.size)
    self.capacity = new_capacity
    free(prev)
end

local function trim_buffer(self)
    if self.size == self.capacity then
        return
    end
    local size = self.size
    local prev = self.bytes
    self.bytes = malloc(size)
    FFI.copy(self.bytes, prev, self.size)
    self.capacity = size
    free(prev)
end

local function count_elements(b)
    local elems = 1
    if _type(b) ~= "number" then
        elems = #b
    end
    return elems
end

local function append(self, b)
    local elems = count_elements(b)
    if self.size + elems > self.capacity then
        grow_buffer(self, elems)
    end
    if _type(b) == "number" then
        self.bytes[self.size] = b
    else
        for i=1, #b do
            self.bytes[self.size + i - 1] = b[i]
        end
    end
    self.size = self.size + elems
end

local function insert(self, index, b)
    if b == nil then
        b = index
        index = self.size + 1
    end
    if index <= 0 or index > self.size + 1 then
        return
    end
    local elems = count_elements(b)
    if self.size + elems > self.capacity then
        grow_buffer(self, elems)
    end
    for i=self.size, index - 1, -1 do
        self.bytes[i + elems] = self.bytes[i]
    end
    if _type(b) == "number" then
        self.bytes[index - 1] = b
    else
        for i=1, #b do
            self.bytes[index + i - 2] = b[i]
        end
    end
    self.size = self.size + elems
end

local function remove(self, index, elems)
    if index <= 0 or index > self.size then
        return
    end
    if elems == nil then
        elems = 1
    end
    if index + elems > self.size then
        elems = self.size - index + 1
    end
    for i=index - 1, self.size - elems - 1 do
        self.bytes[i] = self.bytes[i + elems]
    end
    self.size = self.size - elems
end

local function clear(self)
    self.size = 0
end

local function reserve(self, new_capacity)
    if new_capacity <= self.capacity then
        return
    end
    local prev = self.bytes
    self.bytes = malloc(new_capacity)
    FFI.copy(self.bytes, prev, self.size)
    self.capacity = new_capacity
    free(prev)
end

local function get_capacity(self)
    return self.capacity
end

local bytearray_methods = {
    append=append,
    insert=insert,
    remove=remove,
    trim=trim_buffer,
    clear=clear,
    reserve=reserve,
    get_capacity=get_capacity,
}

local bytearray_mt = {
    __index = function(self, key)
        if _type(key) == "string" then
            return bytearray_methods[key]
        end
        if key <= 0 or key > self.size then
            return
        end
        return self.bytes[key - 1]
    end,
    __newindex = function(self, key, value)
        if key == self.size + 1 then
            return append(self, value)
        elseif key <= 0 or key > self.size then
            return
        end
        self.bytes[key - 1] = value
    end,
    __tostring = function(self)
        return string.format("FFIBytearray[%s]{...}", tonumber(self.size))
    end,
    __len = function(self)
        return tonumber(self.size)
    end,
    __gc = function(self)
        free(self.bytes)
    end,
    __ipairs = function(self)
        local i = 0
        return function()
            i = i + 1
            if i <= self.size then
                return i, self.bytes[i - 1]
            end
        end
    end
}
bytearray_mt.__pairs = bytearray_mt.__ipairs

local bytearray_type = FFI.metatype("bytearray_t", bytearray_mt)

local FFIBytearray = {
    __call = function (self, n)
        local t = type(n)
        if t == "string" then
            local buffer = malloc(#n)
            FFI.copy(buffer, n, #n)
            return bytearray_type(buffer, #n, #n)
        elseif t == "table" then
            local capacity = math.max(#n, MIN_CAPACITY)
            local buffer = FFI.cast("unsigned char*", malloc(capacity))
            for i=1,#n do
                buffer[i - 1] = n[i]
            end
            return bytearray_type(buffer, #n, capacity)
        end
        n = n or 0
        if n < MIN_CAPACITY then
            return bytearray_type(malloc(MIN_CAPACITY), n, MIN_CAPACITY)
        else
            return bytearray_type(malloc(n), n, n)
        end
    end,
}
table.merge(FFIBytearray, bytearray_methods)

local function FFIBytearray_as_string(bytes)
    local t = type(bytes)
    if t == "cdata" then
        return FFI.string(bytes.bytes, bytes.size)
    elseif t == "table" then
        local buffer = FFI.new("unsigned char[?]", #bytes)
        for i=1, #bytes do
            buffer[i - 1] = bytes[i]
        end
        return FFI.string(buffer, #bytes)
    else
        error("Bytearray expected, got "..type(bytes))
    end
end

return {
    FFIBytearray = setmetatable(FFIBytearray, FFIBytearray),
    FFIBytearray_as_string = FFIBytearray_as_string
}
