local rules = {nexid = 1, rules = {}}

function rules.get_rule(name)
    local rule = rules.rules[name]
    if rule == nil then
        rule = {listeners={}}
        rules.rules[name] = rule
    end
    return rule
end

function rules.get(name)
    local rule = rules.rules[name]
    if rule == nil then
        return nil
    end
    return rule.value
end

function rules.set(name, value)
    local rule = rules.get_rule(name)
    rule.value = value
    for _, handler in pairs(rule.listeners) do
        handler(value)
    end
end

function rules.reset(name)
    local rule = rules.get_rule(name)
    rules.set(rule.default)
end

function rules.listen(name, handler)
    local rule = rules.get_rule(name)
    local id = rules.nexid
    rules.nextid = rules.nexid + 1
    rule.listeners[utf8.encode(id)] = handler
    return id
end

function rules.create(name, value, handler)
    local rule = rules.get_rule(name)
    rule.default = value

    local handlerid
    if handler ~= nil then
        handlerid = rules.listen(name, handler)
    end
    if rules.get(name) == nil then
        rules.set(name, value)
    elseif handler then
        handler(rules.get(name))
    end
    return handlerid
end

function rules.unlisten(name, id)
    local rule = rules.rules[name]
    if rule == nil then
        return
    end
    rule.listeners[utf8.encode(id)] = nil
end

function rules.clear()
    rules.rules = {}
    rules.nextid = 1
end

return rules
