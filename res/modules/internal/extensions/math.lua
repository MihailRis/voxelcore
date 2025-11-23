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
