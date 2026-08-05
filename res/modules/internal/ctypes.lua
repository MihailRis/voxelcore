local FFI = ffi

FFI.cdef[[
    unsigned long long strtoull(const char* nptr, char** endptr, int base);
    long long strtoll(const char* nptr, char** endptr, int base);
]]

local this = {}

local function create_integral_type(ctype_name, signed)
    local parse_func = signed and FFI.C.strtoll or FFI.C.strtoull
    return function(value, base)
        value = value or 0
        local typename = type(value)
        if typename == "number" then
            return FFI.cast(ctype_name, value)
        elseif typename == "string" then
            if type(base) ~= "number" then
                base = 10
            end
            return parse_func(value, nil, base)
        end
        error(string.format("invalid value type: %s", typename))
    end
end


this.uint8 = create_integral_type("uint8_t", false)
this.uint16 = create_integral_type("uint16_t", false)
this.uint32 = create_integral_type("uint32_t", false)
this.uint64 = create_integral_type("uint64_t", false)

this.int8 = create_integral_type("int8_t", true)
this.int16 = create_integral_type("int16_t", true)
this.int32 = create_integral_type("int32_t", true)
this.int64 = create_integral_type("int64_t", true)

return this
