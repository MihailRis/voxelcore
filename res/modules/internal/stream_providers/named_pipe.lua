local FFI = ffi

-- named pipes are implemented via the FFI; without a usable one
-- (no LuaJIT / no C symbol access) the feature is unavailable
local ffi_usable = FFI ~= nil and pcall(function()
    FFI.cdef("void* malloc(size_t); void free(void*);")
    FFI.C.free(FFI.C.malloc(8))
end)
if not ffi_usable then
    return function()
        error("named pipes are not available on this platform")
    end
end

if FFI.os == "Windows" then
    return require "core:internal/stream_providers/named_pipe_windows"
else
    return require "core:internal/stream_providers/named_pipe_unix"
end
