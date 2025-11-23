local _ffi = ffi
local _debug_getinfo = debug.getinfo

function crc32(bytes, chksum)
    chksum = chksum or 0

    local length = #bytes
    if type(bytes) == "table" then
        local buffer_len = _ffi.new('int[1]', length)
        local buffer = _ffi.new(
            string.format("char[%s]", length)
        )
        for i=1, length do
            buffer[i - 1] = bytes[i]
        end
        bytes = _ffi.string(buffer, buffer_len[0])
    end
    return _crc32(bytes, chksum)
end

-- Check if given table is an array
function is_array(x)
    if #x > 0 then
        return true
    end
    for k, v in pairs(x) do
        return false
    end
    return true
end

-- Get entry-point and filename from `entry-point:filename` path 
function parse_path(path)
    local index = string.find(path, ':')
    if index == nil then
        error("invalid path syntax (':' missing)")
    end
    return string.sub(path, 1, index-1), string.sub(path, index+1, -1)
end

-- Lua has no parallelizm, also _set_data does not call any lua functions so
-- may be reused one global ffi buffer per lua_State
local canvas_ffi_buffer
local canvas_ffi_buffer_size = 0
local _ffi = ffi
function __vc_Canvas_set_data(self, data)
    if type(data) == "cdata" then
        self:_set_data(tostring(_ffi.cast("uintptr_t", data)))
    end
    local width = self.width
    local height = self.height

    local size = width * height * 4
    if size > canvas_ffi_buffer_size then
        canvas_ffi_buffer = _ffi.new(
            string.format("unsigned char[%s]", size)
        )
        canvas_ffi_buffer_size = size
    end
    for i=0, size - 1 do
        canvas_ffi_buffer[i] = data[i + 1]
    end
    self:_set_data(tostring(_ffi.cast("uintptr_t", canvas_ffi_buffer)))
end

local ipairs_mt_supported = false
for i, _ in ipairs(setmetatable({l={1}}, {
    __ipairs=function(self) return ipairs(self.l) end})) do
    ipairs_mt_supported = true
end

if not ipairs_mt_supported then
    local raw_ipairs = ipairs
    ipairs = function(t)
        local metatable = getmetatable(t)
        if metatable and metatable.__ipairs then
            return metatable.__ipairs(t)
        end
        return raw_ipairs(t)
    end
end

function await(co)
    local res, err
    while coroutine.status(co) ~= 'dead' do
        coroutine.yield()
        res, err = coroutine.resume(co)
        if err then
            return res, err
        end
    end
    return res, err
end

function timeit(iters, func, ...)
    local tm = os.clock()
    for i=1,iters do
        func(...)
    end
    print("[time mcs]", (os.clock()-tm) * 1000000)
end

----------------------------------------------

function debug.count_frames()
    local frames = 1
    while true do
        local info = _debug_getinfo(frames)
        if info then
            frames = frames + 1
        else
            return frames - 1
        end
    end
end

function debug.get_traceback(start)
    local frames = {}
    local n = 2 + (start or 0)
    while true do
        local info = _debug_getinfo(n)
        if info then
            table.insert(frames, info)
        else
            return frames
        end
        n = n + 1
    end
end

package = {
    loaded = {}
}
local __cached_scripts = {}
local __warnings_hidden = {}

function on_deprecated_call(name, alternatives)
    if __warnings_hidden[name] then
        return
    end
    __warnings_hidden[name] = true
    events.emit("core:warning", "deprecated call", name, debug.get_traceback(2))
    if alternatives then
        debug.warning("deprecated function called ("..name.."), use "..
            alternatives.." instead\n"..debug.traceback())
    else
        debug.warning("deprecated function called ("..name..")\n"..debug.traceback())
    end
end

function reload_module(name)
    local prefix, name = parse_path(name)
    local path = prefix..":modules/"..name..".lua"

    local previous = package.loaded[path]
    if not previous then
        debug.log("attempt to reload non-loaded module "..name.." ("..path..")")
        return
    end
    local script, err = load(file.read(path), path)
    if script == nil then
        error(err)
    end
    local result = script()
    if not result then
        return
    end
    for i, value in ipairs(result) do
        previous[i] = value
    end
    local copy = table.copy(result)
    for key, value in pairs(result) do
        result[key] = nil
    end
    for key, value in pairs(copy) do
        previous[key] = value
    end
end

local internal_locked = false

-- Load script with caching
--
-- path - script path `contentpack:filename`. 
--     Example `base:scripts/tests.lua`
--
-- nocache - ignore cached script, load anyway
function __load_script(path, nocache, env)
    local packname, filename = parse_path(path)

    if internal_locked and (packname == "res" or packname == "core") 
       and filename:starts_with("modules/internal") then
        error("access to core:internal modules outside of [core]")
    end

    -- __cached_scripts used in condition because cached result may be nil
    if not nocache and __cached_scripts[path] ~= nil then
        return package.loaded[path]
    end
    if not file.isfile(path) then
        error("script '"..filename.."' not found in '"..packname.."'")
    end

    local script, err = load(file.read(path), path)
    if script == nil then
        error(err)
    end
    if env then
        script = setfenv(script, env)
    end
    local result = script()
    if not nocache then
        __cached_scripts[path] = script
        package.loaded[path] = result
    end
    return result
end

function __vc_lock_internal_modules()
    internal_locked = true
end

function require(path)
    if not string.find(path, ':') then
        local prefix, _ = parse_path(_debug_getinfo(2).source)
        return require(prefix .. ':' .. path)
    end
    local prefix, file = parse_path(path)
    local env = __vc__pack_envs[prefix]
    return __load_script(prefix .. ":modules/" .. file .. ".lua", nil, env)
end

function __scripts_cleanup(non_reset_packs)
    debug.log("cleaning scripts cache")
    if #non_reset_packs == 0 then
        debug.log("no non-reset packs")
    else
        debug.log("non-reset packs: "..table.concat(non_reset_packs, ", "))
    end
    for k, v in pairs(__cached_scripts) do
        local packname, _ = parse_path(k)
        if table.has(non_reset_packs, packname) then
            goto continue
        end
        if packname ~= "core" then
            debug.log("unloaded "..k)
            __cached_scripts[k] = nil
            package.loaded[k] = nil
        end
        __vc__pack_envs[packname] = nil
        ::continue::
    end
end

function __vc__error(msg, frame, n, lastn)
    if events then
        local frames = debug.get_traceback(1)
        events.emit(
            "core:error", msg,
            table.sub(frames, 1 + (n or 0), lastn and #frames-lastn)
        )
    end
    return debug.traceback(msg, frame)
end

function __vc_warning(msg, detail, n)
    if events then
        events.emit(
            "core:warning", msg, detail, debug.get_traceback(1 + (n or 0)))
    end
end

require "core:internal/extensions/pack"
require "core:internal/extensions/math"
require "core:internal/extensions/file"
require "core:internal/extensions/table"
require "core:internal/extensions/string"

local bytearray = require "core:internal/bytearray"
Bytearray = bytearray.FFIBytearray
Bytearray_as_string = bytearray.FFIBytearray_as_string
U16view = bytearray.FFIU16view
I16view = bytearray.FFII16view
U32view = bytearray.FFIU32view
I32view = bytearray.FFII32view
Bytearray_construct = function(...) return Bytearray(...) end

bit.compile = require "core:bitwise/compiler"
bit.execute = require "core:bitwise/executor"

function __vc_create_random_methods(random_methods)
    local index = 1
    local buffer = nil
    local buffer_size = 64

    local seed_func = random_methods.seed
    local random_func = random_methods.random

    function random_methods:bytes(n)
        local bytes = Bytearray(n)
        for i=1,n do
            bytes[i] = self:random(255)
        end
        return bytes
    end

    function random_methods:seed(x)
        seed_func(self, x)
        buffer = nil
    end

    function random_methods:random(a, b)
        if not buffer or index > #buffer then
            buffer = random_func(self, buffer_size)
            index = 1
        end
        local value = buffer[index]
        if b then
            value = math.floor(value * (b - a + 1) + a)
        elseif a then
            value = math.floor(value * a + 1)
        end

        index = index + 4
        return value
    end
    return random_methods
end
