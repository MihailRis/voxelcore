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

function math.sign(x)
    return (x > 0) and 1 or (x < 0 and -1 or 0)
end

local noise_period = 3021
local noise_rands = {}

for i=1,noise_period do
    table.insert(noise_rands, math.random() * 2.0 - 1.0)
end

local function smoothstep(t)
    return t * t * (3 - 2 * t)
end

local function sample_noise(x)
    x = x % noise_period
    local left_cell = math.floor(x)
    local right_cell = (left_cell + 1) % noise_period
    local t = smoothstep(x - left_cell)
    return noise_rands[left_cell + 1] * (1.0 - t)
        + noise_rands[right_cell + 1] * t
end

function math.noise(x, octaves)
    x = math.abs(x)
    octaves = octaves or 1
    local value = 0.0
    local mul = 1.0
    for i=1,octaves do
        value = value * (1.0 - mul) + sample_noise(x + 37 * i) * mul
        x = x * 2.0
        mul = mul * 0.5
    end
    return value
end

local function lerp(a, b, t)
    return a + (b - a) * t
end

local function rand_2d(x, y)
    return noise_rands[math.floor(x * 317 + y) % noise_period + 1]
end

local function sample_noise2d(x, y)
    x = math.abs(x)
    y = math.abs(y)

    local x0 = math.floor(x)
    local y0 = math.floor(y)
    local x1 = x0 + 1
    local y1 = y0 + 1

    local tx = x - x0
    local ty = y - y0

    local sx = smoothstep(tx)
    local sy = smoothstep(ty)

    local c00 = rand_2d(x0, y0)
    local c10 = rand_2d(x1, y0)
    local c01 = rand_2d(x0, y1)
    local c11 = rand_2d(x1, y1)

    local bottom = lerp(c00, c10, sx)
    local top = lerp(c01, c11, sx)

    return lerp(bottom, top, sy)
end


function math.noise2d(x, y, octaves)
    octaves = octaves or 1
    local value = 0.0
    local mul = 1.0
    for i=1,octaves do
        value = value * (1.0 - mul)
            + sample_noise2d(x + 37 * i, y + 73 * i) * mul
        x = x * 2.0
        y = y * 2.0
        mul = mul * 0.5
    end
    return value
end
