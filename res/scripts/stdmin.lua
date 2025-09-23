-- Lua has no parallelizm, also _set_data does not call any lua functions so
-- may be reused one global ffi buffer per lua_State
local canvas_ffi_buffer
local canvas_ffi_buffer_size = 0

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

function crc32(bytes, chksum)
    local chksum = chksum or 0

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

function pack.is_installed(packid)
    return file.isfile(packid..":package.json")
end

function pack.data_file(packid, name)
    file.mkdirs("world:data/"..packid)
    return "world:data/"..packid.."/"..name
end

function pack.shared_file(packid, name)
    file.mkdirs("config:"..packid)
    return "config:"..packid.."/"..name
end


function timeit(iters, func, ...)
    local tm = os.clock()
    for i=1,iters do
        func(...)
    end
    print("[time mcs]", (os.clock()-tm) * 1000000)
end

----------------------------------------------

function math.clamp(_in, low, high)
    return math.min(math.max(_in, low), high)
end

function math.rand(low, high)
    return low + (high - low) * math.random()
end

function math.normalize(num, conf)
    conf = conf or 1

    return (num / conf) % 1
end

function math.round(num, places)
    places = places or 0

    local mult = 10 ^ places
    return math.floor(num * mult + 0.5) / mult
end

function math.sum(...)
    local numbers = nil
    local sum = 0

    if type(...) == "table" then
        numbers = ...
    else
        numbers = {...}
    end

    for _, v in ipairs(numbers) do
        sum = sum + v
    end

    return sum
end

----------------------------------------------

function table.copy(t)
    local copied = {}

    for k, v in pairs(t) do
        copied[k] = v
    end

    return copied
end

function table.deep_copy(t)
    local copied = {}

    for k, v in pairs(t) do
        if type(v) == "table" then
            copied[k] = table.deep_copy(v)
        else
            copied[k] = v
        end
    end

    return setmetatable(copied, getmetatable(t))
end

function table.count_pairs(t)
    local count = 0

    for k, v in pairs(t) do
        count = count + 1
    end

    return count
end

function table.random(t)
    return t[math.random(1, #t)]
end

function table.shuffle(t)
    for i = #t, 2, -1 do
        local j = math.random(i)
        t[i], t[j] = t[j], t[i]
    end

    return t
end

function table.merge(t1, t2)
    for i, v in pairs(t2) do
        if type(i) == "number" then
            t1[#t1 + 1] = v
        elseif t1[i] == nil then
            t1[i] = v
        end
    end

    return t1
end

function table.map(t, func)
    for i, v in pairs(t) do
        t[i] = func(i, v)
    end

    return t
end

function table.filter(t, func)

    for i = #t, 1, -1 do
        if not func(i, t[i]) then
            table.remove(t, i)
        end
    end

    local size = #t

    for i, v in pairs(t) do
        local i_type = type(i)
        if i_type == "number" then
            if i < 1 or i > size then
                if not func(i, v) then
                    t[i] = nil
                end
            end
        else
            if not func(i, v) then
                t[i] = nil
            end
        end
    end

    return t
end

function table.set_default(t, key, default)
    if t[key] == nil then
        t[key] = default
        return default
    end

    return t[key]
end

function table.flat(t)
    local flat = {}

    for _, v in pairs(t) do
        if type(v) == "table" then
            table.merge(flat, v)
        else
            table.insert(flat, v)
        end
    end

    return flat
end

function table.deep_flat(t)
    local flat = {}

    for _, v in pairs(t) do
        if type(v) == "table" then
            table.merge(flat, table.deep_flat(v))
        else
            table.insert(flat, v)
        end
    end

    return flat
end

function table.sub(arr, start, stop)
    local res = {}
    start = start or 1
    stop = stop or #arr

    for i = start, stop do
        table.insert(res, arr[i])
    end

    return res
end

----------------------------------------------

local pattern_escape_replacements = {
    ["("] = "%(",
    [")"] = "%)",
    ["."] = "%.",
    ["%"] = "%%",
    ["+"] = "%+",
    ["-"] = "%-",
    ["*"] = "%*",
    ["?"] = "%?",
    ["["] = "%[",
    ["]"] = "%]",
    ["^"] = "%^",
    ["$"] = "%$",
    ["\0"] = "%z"
}

function string.pattern_safe(str)
    return string.gsub(str, ".", pattern_escape_replacements)
end

local string_sub = string.sub
local string_find = string.find
local string_len = string.len
function string.explode(separator, str, withpattern)
    if (withpattern == nil) then withpattern = false end

    local ret = {}
    local current_pos = 1

    for i = 1, string_len(str) do
        local start_pos, end_pos = string_find(
            str, separator, current_pos, not withpattern)
        if (not start_pos) then break end
        ret[i] = string_sub(str, current_pos, start_pos - 1)
        current_pos = end_pos + 1
    end

    ret[#ret + 1] = string_sub(str, current_pos)

    return ret
end

function string.split(str, delimiter)
    return string.explode(delimiter, str)
end

function string.formatted_time(seconds, format)
    if (not seconds) then seconds = 0 end
    local hours = math.floor(seconds / 3600)
    local minutes = math.floor((seconds / 60) % 60)
    local millisecs = (seconds - math.floor(seconds)) * 1000
    seconds = math.floor(seconds % 60)

    if (format) then
        return string.format(format, minutes, seconds, millisecs)
    else
        return { h = hours, m = minutes, s = seconds, ms = millisecs }
    end
end

function string.replace(str, tofind, toreplace)
    local tbl = string.explode(tofind, str)
    if (tbl[1]) then return table.concat(tbl, toreplace) end
    return str
end

function string.trim(s, char)
    if char then char = string.pattern_safe(char) else char = "%s" end
    return string.match(s, "^" .. char .. "*(.-)" .. char .. "*$") or s
end

function string.trim_right(s, char)
    if char then char = string.pattern_safe(char) else char = "%s" end
    return string.match(s, "^(.-)" .. char .. "*$") or s
end

function string.trim_left(s, char)
    if char then char = string.pattern_safe(char) else char = "%s" end
    return string.match(s, "^" .. char .. "*(.+)$") or s
end

function string.pad(str, size, char)
    char = char == nil and " " or char

    local padding = math.floor((size - #str) / 2)
    local extra_padding = (size - #str) % 2

    return string.rep(char, padding) .. str .. string.rep(char, padding + extra_padding)
end

function string.left_pad(str, size, char)
    char = char == nil and " " or char

    local left_padding = size - #str
    return string.rep(char, left_padding) .. str
end

function string.right_pad(str, size, char)
    char = char == nil and " " or char

    local right_padding = size - #str
    return str .. string.rep(char, right_padding)
end

string.lower = utf8.lower
string.upper = utf8.upper
string.escape = utf8.escape

local meta = getmetatable("")

function meta:__index(key)
    local val = string[key]
    if (val ~= nil) then
        return val
    elseif (tonumber(key)) then
        return string.sub(self, key, key)
    end
end

function string.starts_with(str, start)
    return string.sub(str, 1, string.len(start)) == start
end

function string.ends_with(str, endStr)
    return endStr == "" or string.sub(str, -string.len(endStr)) == endStr
end

function table.has(t, x)
    for i,v in ipairs(t) do
        if v == x then
            return true
        end
    end
    return false
end

function table.index(t, x)
    for i,v in ipairs(t) do
        if v == x then
            return i
        end
    end
    return -1
end

function table.remove_value(t, x)
    local index = table.index(t, x)
    if index ~= -1 then
        table.remove(t, index)
    end
end

function table.tostring(t)
    local s = '['
    for i,v in ipairs(t) do
        s = s..tostring(v)
        if i < #t then
            s = s..', '
        end
    end
    return s..']'
end

function file.readlines(path)
    local str = file.read(path)
    local lines = {}
    for s in str:gmatch("[^\r\n]+") do
        table.insert(lines, s)
    end
    return lines
end

local _debug_getinfo = debug.getinfo

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
function __load_script(path, nocache)
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
        return require(prefix..':'..path)
    end
    local prefix, file = parse_path(path)
    return __load_script(prefix..":modules/"..file..".lua")
end

function __scripts_cleanup()
    debug.log("cleaning scripts cache")
    for k, v in pairs(__cached_scripts) do
        local packname, _ = parse_path(k)
        if packname ~= "core" then
            debug.log("unloaded "..k)
            __cached_scripts[k] = nil
            package.loaded[k] = nil
        end
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

function file.name(path)
    return path:match("([^:/\\]+)$")
end

function file.stem(path)
    local name = file.name(path)
    return name:match("(.+)%.[^%.]+$") or name
end

function file.ext(path)
    return path:match("%.([^:/\\]+)$")
end

function file.prefix(path)
    return path:match("^([^:]+)")
end

function file.parent(path)
    local dir = path:match("(.*)/")
    if not dir then
        return file.prefix(path)..":"
    end
    return dir
end

function file.path(path)
    local pos = path:find(':')
    return path:sub(pos + 1)
end

function file.join(a, b)
    if a[#a] == ':' then
        return a .. b
    end
    return a .. "/" .. b
end

bit.compile = require "core:bitwise/compiler"
bit.execute = require "core:bitwise/executor"

random.Random = require "core:internal/random_generator"
