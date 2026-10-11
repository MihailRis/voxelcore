local wrapper = require "internal/stream/wrapper"

local blocking = wrapper.extend({})

function blocking.new(inner)
    return setmetatable({ inner = inner }, blocking)
end

function blocking:read(n)
    local result = self.inner:read(n)
    while #result < n do
        coroutine.yield()
        if not self.inner:is_alive() then break end
        result:append(self.inner:read(n - #result))
    end
    return result
end

return blocking
