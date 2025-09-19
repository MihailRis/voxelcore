local Random = {}

local M = 2 ^ 31
local A = 1103515245
local C = 12345

function Random.randint(self)
    self._seed = (A * self._seed + C) % M
    return self._seed
end

function Random.random(self, a, b)
    local num = self:randint() % M / M
    if b then
        return math.floor(num * (b - a + 1) + a)
    elseif a then
        return math.floor(num * a + 1)
    else
        return num
    end
end

function Random.seed(self, number)
    if type(number) ~= "number" then
        error("number expected")
    end
    self._seed = number
end

return function(seed)
    if seed and type(seed) ~= "number" then
        error("number expected")
    end
    return setmetatable({_seed = seed or random.random(M)}, {__index = Random})
end
