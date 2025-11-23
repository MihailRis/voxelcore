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

function file.readlines(path)
    local str = file.read(path)
    local lines = {}
    for s in str:gmatch("[^\r\n]+") do
        table.insert(lines, s)
    end
    return lines
end
