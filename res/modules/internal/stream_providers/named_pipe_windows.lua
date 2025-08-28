local path_validate = require "core:internal/stream_providers/named_pipe_path_validate"
local io_stream = require "core:io_stream"

local FFI = ffi

FFI.cdef[[
typedef void* HANDLE;
typedef uint32_t DWORD;
typedef int BOOL;
typedef void* LPVOID;
typedef const char* LPCSTR;

BOOL CloseHandle(HANDLE hObject);
DWORD GetFileType(HANDLE hFile);
BOOL ReadFile(HANDLE hFile, void* lpBuffer, DWORD nNumberOfBytesToRead,
              DWORD* lpNumberOfBytesRead, void* lpOverlapped);
BOOL WriteFile(HANDLE hFile, const void* lpBuffer, DWORD nNumberOfBytesToWrite,
               DWORD* lpNumberOfBytesWritten, void* lpOverlapped);
HANDLE CreateFileA(LPCSTR lpFileName, DWORD dwDesiredAccess, DWORD dwShareMode,
                   void* lpSecurityAttributes, DWORD dwCreationDisposition,
                   DWORD dwFlagsAndAttributes, HANDLE hTemplateFile);

BOOL PeekNamedPipe(
    HANDLE hNamedPipe,
    LPVOID lpBuffer,
    DWORD nBufferSize,
    DWORD* lpBytesRead,
    DWORD* lpTotalBytesAvail,
    DWORD* lpBytesLeftThisMessage
);

DWORD GetLastError(void);
BOOL FlushFileBuffers(HANDLE hFile);
]]

local C = FFI.C

local GENERIC_READ  = 0x80000000
local GENERIC_WRITE = 0x40000000
local OPEN_EXISTING = 3
local FILE_ATTRIBUTE_NORMAL = 0x00000080
local FILE_TYPE_UNKNOWN = 0x0000
local INVALID_HANDLE_VALUE = FFI.cast("HANDLE", -1)

local lib = {}

local function is_data_available(handle)
    local bytes_available = FFI.new("DWORD[1]")
    local success = FFI.C.PeekNamedPipe(handle, nil, 0, nil, bytes_available, nil)

    if success == 0 then
        return -1
    end

    return bytes_available[0] > 0
end

function lib.read(handle, len)
    local out = Bytearray()

    local has_data, err = is_data_available(handle)

    if not has_data then
        return out
    elseif hasData == -1 then
        error("failed to read from named pipe: "..tostring(C.GetLastError()))
    end

    local buffer = FFI.new("uint8_t[?]", len)
    local read = FFI.new("DWORD[1]")

    local ok = C.ReadFile(handle, buffer, len, read, nil)

    if ok == 0 or read[0] == 0 then
        return out
    end

    for i = 0, read[0] - 1 do
        out[i+1] = buffer[i]
    end

    return out
end

function lib.write(handle, bytearray)
    local len = #bytearray
    
    local buffer = FFI.new("uint8_t[?]", len)
    for i = 1, len do
        buffer[i-1] = bytearray[i]
    end

    local written = FFI.new("DWORD[1]")

    if C.WriteFile(handle, buffer, len, written, nil) == 0 then
        error("failed to write to named pipe: "..tostring(C.GetLastError()))
    end
end

function lib.flush(handle)
    C.FlushFileBuffers(handle)
end

function lib.is_alive(handle)
    if handle == nil or handle == INVALID_HANDLE_VALUE then
        return false
    else
        return C.GetFileType(handle) ~= FILE_TYPE_UNKNOWN
    end
end

function lib.close(handle)
    C.CloseHandle(handle)
end

return function(path, mode)
    path_validate(path)

    path = "\\\\.\\pipe\\"..path

    local read = mode:find('r') ~= nil
    local write = mode:find('w') ~= nil

    local flags

    if read and write then
        flags = bit.bor(GENERIC_READ, GENERIC_WRITE)
    elseif read then
        flags = GENERIC_READ
    elseif write then
        flags = GENERIC_WRITE
    else
        error("mode must contain read or write flag")
    end

    local handle = C.CreateFileA(path, flags, 0, nil, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, nil)

    if handle == INVALID_HANDLE_VALUE then
        error("failed to open named pipe: "..tostring(C.GetLastError()))
    end

    return io_stream.new(handle, mode:find('b') ~= nil, lib)
end