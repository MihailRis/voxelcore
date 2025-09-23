local FFI = ffi

if FFI.os == "Windows" then
    return require "core:internal/stream_providers/named_pipe_windows"
else
    return require "core:internal/stream_providers/named_pipe_unix"
end