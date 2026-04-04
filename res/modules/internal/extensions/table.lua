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

function table.insert_unique(t, pos_or_val, val)
    if table.has(t, val or pos_or_val) then
        return
    end

    if val then
        table.insert(t, pos_or_val, val)
    else
        table.insert(t, pos_or_val)
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
