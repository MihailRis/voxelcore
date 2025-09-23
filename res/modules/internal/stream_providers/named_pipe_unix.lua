local path_validate = require "core:internal/stream_providers/named_pipe_path_validate"
local io_stream = require "core:io_stream"

local FFI = ffi

FFI.cdef[[
int open(const char *pathname, int flags);
int close(int fd);
ssize_t read(int fd, void *buf, size_t count);
ssize_t write(int fd, const void *buf, size_t count);
int fcntl(int fd, int cmd, ...);

const char *strerror(int errnum);
]]

local C = FFI.C

local O_RDONLY   = 0x0
local O_WRONLY   = 0x1
local O_RDWR     = 0x2
local O_NONBLOCK = 0x800
local F_GETFL    = 3

local function getError()
    local err = FFI.errno()

    return FFI.string(C.strerror(err)).." ("..err..")"
end

local lib = {}

function lib.read(fd, len)
    local buffer = FFI.new("uint8_t[?]", len)
    local result = tonumber(C.read(fd, buffer, len))

    local out = Bytearray()

    if result <= 0 then
        return out
    end

    for i = 0, result - 1 do
        out[i+1] = buffer[i]
    end

    return out
end

function lib.write(fd, bytearray)
    local len = #bytearray
    local buffer = FFI.new("uint8_t[?]", len)
    for i = 1, len do
        buffer[i-1] = bytearray[i]
    end

    if C.write(fd, buffer, len) == -1 then
        error("failed to write to named pipe: "..getError())
    end
end

function lib.flush(fd)
    -- no flush on unix
end

function lib.is_alive(fd)
    if fd == nil or fd < 0 then return false end

    return C.fcntl(fd, F_GETFL) ~= -1
end

function lib.close(fd)
    C.close(fd)
end

return function(path, mode)
    path_validate(path)

    path = "/tmp/"..path

    local read = mode:find('r') ~= nil
    local write = mode:find('w') ~= nil

    local flags

    if read and write then
        flags = O_RDWR
    elseif read then
        flags = O_RDONLY
    elseif write then
        flags = O_WRONLY
    else
        error "mode must contain read or write flag"
    end

    flags = bit.bor(flags, O_NONBLOCK)

    local fd = C.open(path, flags)

    if fd == -1 then
        error("failed to open named pipe: "..getError())
    end

    return io_stream.new(fd, mode:find('b') ~= nil, lib)
end
