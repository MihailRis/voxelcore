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
