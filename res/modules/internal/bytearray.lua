local MIN_CAPACITY = 8
local _type = type
local FFI = ffi

FFI.cdef[[
    void* malloc(size_t);
    void* realloc(void*, size_t);
    void free(void*);
    typedef struct {
        unsigned char* bytes;
        int size;
        int capacity;
    } bytearray_t;
    void* memmove(void*, const void*, size_t);
]]

local free = FFI.C.free

local FFIBytearray
local bytearray_type

local allocated_bytes = 0
local GC_THRESHOLD = 50 * 1024 * 1024

local function malloc(size)
    local raw = FFI.C.malloc(size)
    if not raw then
        error("failed memory allocation of " .. size .. " bytes")
    end

    allocated_bytes = allocated_bytes + size
    if allocated_bytes >= GC_THRESHOLD then
        collectgarbage("step", 200)
        allocated_bytes = 0
    end

    return raw
end

local function realloc(buff, size, throwerror)
    if throwerror == nil then throwerror = true end

    if size <= 0 then
        free(buff)
        return
    end

    local raw = FFI.C.realloc(buff, size)
    if not raw and throwerror then
        error("failed memory reallocation of block " .. buff .. " to new size " .. size)
    end

    allocated_bytes = allocated_bytes + size
    if allocated_bytes >= GC_THRESHOLD then
        collectgarbage("step", 200)
        allocated_bytes = 0
    end

    return raw
end

local function grow_buffer(self, elems)
    local new_capacity = math.ceil(self.capacity / 0.75 + elems)
    self.bytes = realloc(self.bytes, new_capacity)
    self.capacity = new_capacity
end

local function trim_buffer(self)
    if self.size == self.capacity then return end
    local new_bytes = realloc(self.bytes, self.size, false)
    if new_bytes then
        self.bytes = new_bytes
        self.capacity = self.size
    end
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
    for i = self.size - 1, index - 1, -1 do
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

local function fill(self, index, size, byte)
    if index ~= nil and size ~= nil and size > 0 then
        if index < 1 or index + size - 1 > self.size then
            error("selected fill range is out of array bounds")
        end
        FFI.fill(self.bytes + index - 1, size, byte)
    else
        FFI.fill(self.bytes, self.size, byte)
    end
end

local function reserve(self, new_capacity)
    if new_capacity <= self.capacity then return end
    self.bytes = realloc(self.bytes, new_capacity)
    self.capacity = new_capacity
end

local function get_capacity(self)
    return self.capacity
end

local function slice(self, offset, length)
    offset = offset or 1
    length = length or (self.size - offset + 1)
    if offset < 1 or offset > self.size then
        return FFIBytearray(0)
    end
    if offset + length - 1 > self.size then
        length = self.size - offset + 1
    end
    local buffer = malloc(length)
    FFI.copy(buffer, self.bytes + (offset - 1), length)
    return bytearray_type(buffer, length, length)
end

local function copy(self, srcindex, dst, dstindex, size)
    if size <= 0 then error("size of byte range must be positive non-zero integer") end
    if srcindex < 1 or srcindex + size - 1 > self.size then
        error("specified source byte range is out of source array bounds")
    end
    if dstindex < 1 or dstindex + size - 1 > dst.size then
        error("specified destination byte range is out of destination array bounds")
    end

    FFI.copy(dst.bytes + dstindex - 1, self.bytes + srcindex - 1, size)
end

local function move(self, fromindex, toindex, size)
    if size <= 0 then error("size of byte range must be positive non-zero integer") end
    if fromindex < 1 or fromindex + size - 1 > self.size then
        error("specified source byte range is out of array bounds")
    end
    if toindex < 1 or toindex + size - 1 > self.size then
        error("specified destination byte range is out of array bounds")
    end

    FFI.C.memmove(self.bytes + toindex - 1, self.bytes + fromindex - 1, size)
end

local bytearray_methods = {
    append = append,
    insert = insert,
    remove = remove,
    trim = trim_buffer,
    clear = clear,
    reserve = reserve,
    get_capacity = get_capacity,
    slice = slice,
    fill = fill,
    copy = copy,
    move = move
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

bytearray_type = FFI.metatype("bytearray_t", bytearray_mt)

FFIBytearray = {
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

local function FFIBytearray_as_ptr(bytes)
    if type(bytes) == "cdata" then
        return tostring(bytes.bytes):sub(27), bytes.size
    end
    return "0"
end

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

local function create_FFIview_class(name, typename)
    local typesize = FFI.sizeof(typename)
    local ptrtype = typename .. "*"
    local FFIview_mt = {
        __index = function(self, key)
            if key == 'size' then
                return self.bytes.size / typesize
            end
            if key <= 0 or key > self.bytes.size / typesize then
                return
            end
            local ptr = FFI.cast(ptrtype, self.bytes.bytes)
            return ptr[key - 1]
        end,
        __newindex = function(self, key, value)
            if key <= 0 or key > self.bytes.size / typesize then
                return
            end
            local ptr = FFI.cast(ptrtype, self.bytes.bytes)
            ptr[key - 1] = value
        end,
        __len = function(self)
            return self.bytes.size / typesize
        end,
        __tostring = function(self)
            return string.format(name .. "[%s]{...}", tonumber(self.bytes.size / typesize))
        end,
        __ipairs = function(self)
            local i = 0
            return function()
                i = i + 1
                if i <= self.bytes.size / typesize then
                    local ptr = FFI.cast(ptrtype, self.bytes.bytes)
                    return i, ptr[i - 1]
                end
            end
        end,
    }
    return setmetatable({}, {
        __call = function (self, bytes)
            local x = setmetatable({
                bytes=bytes,
            }, FFIview_mt)
            return x
        end,
        __index = {
            typesize = function()
                return typesize
            end
        }
    })
end

local FFII8view = create_FFIview_class("FFII8view", "int8_t")
local FFII16view = create_FFIview_class("FFII16view", "int16_t")
local FFIU16view = create_FFIview_class("FFIU16view", "uint16_t")
local FFII32view = create_FFIview_class("FFII32view", "int32_t")
local FFIU32view = create_FFIview_class("FFIU32view", "uint32_t")
local FFII64view = create_FFIview_class("FFII64view", "int64_t")
local FFIU64view = create_FFIview_class("FFIU64view", "uint64_t")
local FFIFLTview = create_FFIview_class("FFIFLTview", "float")
local FFIDBLview = create_FFIview_class("FFIDBLview", "double")

return {
    FFIBytearray = setmetatable(FFIBytearray, FFIBytearray),
    FFIBytearray_as_string = FFIBytearray_as_string,
    FFIBytearray_as_ptr = FFIBytearray_as_ptr,
    FFII8view = FFII8view,
    FFIU16view = FFIU16view,
    FFII16view = FFII16view,
    FFIU32view = FFIU32view,
    FFII32view = FFII32view,
    FFIU64view = FFIU64view,
    FFII64view = FFII64view,
    FFIFLTview = FFIFLTview,
    FFIDBLview = FFIDBLview,
}
