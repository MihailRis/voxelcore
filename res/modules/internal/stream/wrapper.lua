local wrapper = {}
wrapper.__index = wrapper

function wrapper.extend(cls)
    cls.__index = cls
    return setmetatable(cls, { __index = wrapper })
end
function wrapper:read(n) return self.inner:read(n) end

function wrapper:read_some(n) return self.inner:read_some(n) end

function wrapper:write(data) return self.inner:write(data) end

function wrapper:flush() return self.inner:flush() end

function wrapper:available() return self.inner:available() end

function wrapper:seek(mode, off) return self.inner:seek(mode, off) end

function wrapper:tell() return self.inner:tell() end

function wrapper:is_alive() return self.inner:is_alive() end

function wrapper:close() return self.inner:close() end

return wrapper
