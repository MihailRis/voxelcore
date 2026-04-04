local entries = {}
local this = {}

function this.get(name)
    local entry = entries[name]
    if entry == nil then
        entry = {}
        entries[name] = entry
    end
    return entry
end

function this.has(name)
    return entries[name] ~= nil
end

function this.reset(name)
    entries[name] = nil
end

return this
